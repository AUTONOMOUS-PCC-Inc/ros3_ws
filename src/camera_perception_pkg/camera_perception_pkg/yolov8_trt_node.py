#!/usr/bin/env python3
# TensorRT 기반 YOLOv8-seg 노드: /image_raw -> TRT(GPU) -> /detections (DetectionArray)
# ultralytics/torchvision 불필요. 기존 lane_info_extractor가 /detections를 받아 /roi_image 발행.
import time, numpy as np, cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import tensorrt as trt
from cuda.bindings import runtime as cudart
from interfaces_pkg.msg import (DetectionArray, Detection, Mask, BoundingBox2D, Point2D)

import os
ENGINE = os.path.expanduser("~/ros3_ws/best.engine")
NAMES = {0: "lane"}
CONF, IOU = 0.25, 0.45

def ck(e):
    e = e[0] if isinstance(e, tuple) else e
    if int(e) != 0: raise RuntimeError(f"CUDA err {e}")

class TRTSeg:
    def __init__(self, path):
        lg = trt.Logger(trt.Logger.ERROR)
        with open(path, "rb") as f, trt.Runtime(lg) as rt:
            self.engine = rt.deserialize_cuda_engine(f.read())
        self.ctx = self.engine.create_execution_context()
        err, self.stream = cudart.cudaStreamCreate(); ck(err)
        self.inp, self.outs = None, []
        for i in range(self.engine.num_io_tensors):
            n = self.engine.get_tensor_name(i)
            shp = tuple(self.ctx.get_tensor_shape(n))
            dt = trt.nptype(self.engine.get_tensor_dtype(n))
            nb = int(np.prod(shp)) * np.dtype(dt).itemsize
            err, dptr = cudart.cudaMalloc(nb); ck(err)
            self.ctx.set_tensor_address(n, int(dptr))
            rec = dict(shape=shp, nbytes=nb, dptr=dptr, host=np.empty(shp, dt))
            if self.engine.get_tensor_mode(n) == trt.TensorIOMode.INPUT: self.inp = rec
            else: self.outs.append(rec)
        self.S = self.inp["shape"][2]

    def infer(self, chw):
        np.copyto(self.inp["host"], chw[None])
        ck(cudart.cudaMemcpyAsync(self.inp["dptr"], self.inp["host"].ctypes.data, self.inp["nbytes"],
                                  cudart.cudaMemcpyKind.cudaMemcpyHostToDevice, self.stream))
        self.ctx.execute_async_v3(self.stream)
        for o in self.outs:
            ck(cudart.cudaMemcpyAsync(o["host"].ctypes.data, o["dptr"], o["nbytes"],
                                      cudart.cudaMemcpyKind.cudaMemcpyDeviceToHost, self.stream))
        ck(cudart.cudaStreamSynchronize(self.stream))
        return [o["host"] for o in self.outs]

def sigmoid(x): return 1/(1+np.exp(-x))

def letterbox(img, S):
    h,w = img.shape[:2]; r = min(S/h, S/w)
    nw,nh = int(round(w*r)), int(round(h*r))
    canvas = np.full((S,S,3),114,np.uint8)
    top,left = (S-nh)//2,(S-nw)//2
    canvas[top:top+nh, left:left+nw] = cv2.resize(img,(nw,nh))
    return canvas, left, top

class TRTNode(Node):
    def __init__(self):
        super().__init__("yolov8_trt_node")
        self.net = TRTSeg(ENGINE)
        self.br = CvBridge()
        self.pub = self.create_publisher(DetectionArray, "detections", 10)
        self.sub = self.create_subscription(Image, "image_raw", self.cb, 1)
        self.t, self.n = time.time(), 0
        self.get_logger().info("TRT YOLOv8-seg node ready (GPU, torch-free)")

    def cb(self, msg):
        img = self.br.imgmsg_to_cv2(msg, desired_encoding="bgr8") if msg.encoding=="bgr8" \
              else self.br.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        if img.ndim==2: img = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
        oh,ow = img.shape[:2]; S = self.net.S
        lb,left,top = letterbox(img,S)
        chw = np.ascontiguousarray(cv2.cvtColor(lb,cv2.COLOR_BGR2RGB).transpose(2,0,1)/255.0, dtype=np.float32)
        outs = self.net.infer(chw)
        det_msg = self.build(outs, left, top, ow, oh)
        det_msg.header = msg.header
        self.pub.publish(det_msg)
        self.n += 1
        if time.time()-self.t >= 3.0:
            self.get_logger().info(f"{self.n/(time.time()-self.t):.1f} FPS | dets/frame last={len(det_msg.detections)}")
            self.t, self.n = time.time(), 0

    def build(self, outs, left, top, ow, oh):
        m = DetectionArray()
        det = next(o for o in outs if o.ndim==3 and o.shape[1]==37)[0].T  # (8400,37)
        proto = next(o for o in outs if o.ndim==4)[0]                      # (32,160,160)
        sc = det[:,4]; keep = sc>CONF
        det, sc = det[keep], sc[keep]
        if len(det)==0: return m
        b = det[:,:4]; xy = np.stack([b[:,0]-b[:,2]/2, b[:,1]-b[:,3]/2, b[:,0]+b[:,2]/2, b[:,1]+b[:,3]/2],1)
        idx = cv2.dnn.NMSBoxes(xy.tolist(), sc.tolist(), CONF, IOU)
        if len(idx)==0: return m
        idx = np.array(idx).flatten()
        ph,pw = proto.shape[1:]
        masks = sigmoid(det[idx,5:] @ proto.reshape(32,-1)).reshape(-1,ph,pw)
        S = self.net.S
        for j,di in enumerate(idx):
            mk = (cv2.resize(masks[j],(S,S))>0.5).astype(np.uint8)
            x1,y1,x2,y2 = np.clip(xy[di],0,S).astype(int)
            crop = np.zeros_like(mk); crop[y1:y2,x1:x2]=mk[y1:y2,x1:x2]; mk=crop
            mk = mk[top:S-top if top else S, left:S-left if left else S]
            mk = cv2.resize(mk,(ow,oh),interpolation=cv2.INTER_NEAREST)
            cnts,_ = cv2.findContours(mk, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            if not cnts: continue
            c = max(cnts, key=cv2.contourArea)
            d = Detection(); d.class_id=0; d.class_name="lane"; d.score=float(sc[di])
            bb = BoundingBox2D()
            bb.center.position.x=float((x1+x2)/2 - left)*ow/(S-2*left if left else S)
            bb.center.position.y=float((y1+y2)/2 - top)*oh/(S-2*top if top else S)
            bb.size.x=float(x2-x1); bb.size.y=float(y2-y1)
            d.bbox=bb
            mask=Mask(); mask.height=oh; mask.width=ow
            mask.data=[Point2D(x=float(p[0][0]), y=float(p[0][1])) for p in c]
            d.mask=mask
            m.detections.append(d)
        return m

def main():
    rclpy.init(); n=TRTNode()
    try: rclpy.spin(n)
    except KeyboardInterrupt: pass
    finally: n.destroy_node(); rclpy.shutdown()

if __name__=="__main__": main()
