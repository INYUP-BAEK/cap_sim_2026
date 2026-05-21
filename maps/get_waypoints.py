import cv2
import yaml
import math

# 1. 파일 경로 설정
yaml_path = '320.yaml'
image_path = '320.png'

# 2. YAML 파일에서 변환 정보 읽기
with open(yaml_path, 'r') as f:
    map_data = yaml.safe_load(f)

resolution = map_data['resolution']
origin_x = map_data['origin'][0]
origin_y = map_data['origin'][1]

# 3. 이미지 로드
img = cv2.imread(image_path)
height, width = img.shape[:2]
img_copy = img.copy()
start_pt = None

# 마우스 드래그를 통한 위치 및 방향 계산 함수
def get_pose(event, x, y, flags, param):
    global start_pt, img_copy

    # 마우스 누를 때 (시작점 = 로봇 위치)
    if event == cv2.EVENT_LBUTTONDOWN:
        start_pt = (x, y)

    # 마우스 뗄 때 (끝점 = 바라볼 방향)
    elif event == cv2.EVENT_LBUTTONUP:
        if start_pt is None: return
        end_pt = (x, y)

        # 1. 픽셀 -> 미터 좌표 변환 (위치)
        map_x = origin_x + (start_pt[0] * resolution)
        map_y = origin_y + ((height - start_pt[1]) * resolution)

        # 2. 픽셀 변화량으로 실제 맵 상의 방향(각도) 계산
        # (이미지의 Y축은 아래로 향하므로 실제 좌표계에 맞춰 반전)
        dx = (end_pt[0] - start_pt[0]) * resolution
        dy = -(end_pt[1] - start_pt[1]) * resolution

        # atan2를 이용해 라디안(Radian) 각도(Yaw) 추출
        yaw = math.atan2(dy, dx)

        # 3. Yaw 각도를 ROS 2 Nav2용 Quaternion으로 변환
        qz = math.sin(yaw / 2.0)
        qw = math.cos(yaw / 2.0)

        degrees = math.degrees(yaw)
        print(f"📍 위치: X={map_x:.2f}, Y={map_y:.2f} | 🧭 방향: {degrees:.1f}도 | 🔄 쿼터니언: z={qz:.3f}, w={qw:.3f}")

        # 화면에 빨간색 화살표 그리기
        cv2.arrowedLine(img_copy, start_pt, end_pt, (0, 0, 255), 2, tipLength=0.3)
        cv2.imshow('Map Waypoint Picker', img_copy)
        start_pt = None

cv2.imshow('Map Waypoint Picker', img_copy)
cv2.setMouseCallback('Map Waypoint Picker', get_pose)
print("마우스 왼쪽 버튼을 '누른 상태로 드래그'하여 방향을 설정하세요. (종료: ESC 키)")

cv2.waitKey(0)
cv2.destroyAllWindows()
