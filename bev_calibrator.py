#!/usr/bin/env python3
"""
BEV 캘리브레이션 도구 (HL FMA / aMAP 1/5)
- 라이브 /image_raw 한 프레임(또는 이미지 파일)에서 마우스로 차선영역 4점을 찍으면
  실제 코드와 동일한 dst_mat 공식으로 BEV를 미리보고, 붙여넣을 src_mat을 출력.
- 목표: BEV에서 좌우 차선이 '수직 평행'이 되도록 4점을 조정 → 조향 정확도↑

사용법:
  python3 bev_calibrator.py            # 라이브 /image_raw 캡처
  python3 bev_calibrator.py frame.png  # 저장된 이미지 사용

조작:
  좌클릭 4번 = 사다리꼴 4점 (순서: 좌상→우상→우하→좌하)
  r = 리셋,  s = src_mat 출력/저장,  q = 종료
반영 위치(2곳 동일하게):
  src/camera_perception_pkg/.../lane_info_extractor_node.py  (src_mat)
  src/camera_perception_pkg/.../lib/camera_perception_func_lib.py (edge_image_postproc의 src_mat)
"""
import sys, cv2, numpy as np

CUT = 300  # roi_rectangle_below cutting_idx (코드와 동일)

def get_frame():
    if len(sys.argv) > 1:
        img = cv2.imread(sys.argv[1])
        if img is None: sys.exit(f"이미지 못 읽음: {sys.argv[1]}")
        return img
    # 라이브 캡처
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import Image
    from cv_bridge import CvBridge
    rclpy.init(); n = Node("bev_cal"); br = CvBridge(); g = {}
    n.create_subscription(Image, "/image_raw",
        lambda m: g.__setitem__('f', br.imgmsg_to_cv2(m, desired_encoding="passthrough")), 1)
    import time; t = time.time()
    while 'f' not in g and time.time()-t < 8:
        rclpy.spin_once(n, timeout_sec=0.2)
    rclpy.shutdown()
    if 'f' not in g: sys.exit("/image_raw 프레임 못 받음 (launch/카메라 확인)")
    f = g['f']
    return f if f.ndim == 3 else cv2.cvtColor(f, cv2.COLOR_GRAY2BGR)

def dst_mat(w, h):
    return np.float32([[round(w*0.3),0],[round(w*0.7),0],[round(w*0.7),h],[round(w*0.3),h]])

pts = []
def on_mouse(ev, x, y, flags, param):
    if ev == cv2.EVENT_LBUTTONDOWN and len(pts) < 4:
        pts.append([x, y])

def main():
    img = get_frame()
    h, w = img.shape[:2]
    cv2.namedWindow("calib"); cv2.setMouseCallback("calib", on_mouse)
    print(f"[i] 프레임 {w}x{h}. 좌상→우상→우하→좌하 순서로 4점 클릭.")
    labels = ["1 좌상","2 우상","3 우하","4 좌하"]
    while True:
        vis = img.copy()
        for i,p in enumerate(pts):
            cv2.circle(vis,tuple(p),5,(0,255,255),-1)
            cv2.putText(vis,labels[i],(p[0]+6,p[1]),0,0.5,(0,255,255),1)
        if len(pts) >= 2:
            cv2.polylines(vis,[np.int32(pts)],len(pts)==4,(0,0,255),2)
        cv2.putText(vis,f"pts {len(pts)}/4  [r]eset [s]ave [q]uit",(10,25),0,0.6,(255,255,255),2)
        cv2.imshow("calib", vis)
        if len(pts) == 4:
            src = np.float32(pts)
            M = cv2.getPerspectiveTransform(src, dst_mat(w,h))
            bird = cv2.warpPerspective(img, M, (w,h))
            roi = bird[CUT:]
            grid = bird.copy()
            for gx in range(0,w,40): cv2.line(grid,(gx,0),(gx,h),(0,255,0),1)
            cv2.imshow("BEV (수직선이 차선과 평행하도록 4점 조정)", grid)
            cv2.imshow("ROI(below %d)"%CUT, roi)
        k = cv2.waitKey(20) & 0xFF
        if k == ord('r'): pts.clear()
        elif k == ord('q'): break
        elif k == ord('s') and len(pts) == 4:
            s = [[int(p[0]),int(p[1])] for p in pts]
            print("\n=== 아래 src_mat을 두 파일에 동일하게 붙여넣기 ===")
            print(f"src_mat = {s}")
            with open("/home/acelab/ros3_ws/bev_src_mat.txt","w") as f:
                f.write(f"src_mat = {s}\n")
            print("(bev_src_mat.txt 로도 저장됨)\n")
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
