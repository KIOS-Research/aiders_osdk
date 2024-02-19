import cv2
import math

sample_dir = 'qr_samples/'
window_name = 'OpenCV QR Code'

def qr_detector():
    print('Starting QR Detector')
    qcd = cv2.QRCodeDetector()

    img = cv2.imread(sample_dir + 'qr_code_topleft.png')

    if img.any():
        print('cv2.imread success')

    retval, decoded_info, points, straight_qrcode = qcd.detectAndDecodeMulti(img)
    
    if retval:
        print('Detection Success')
        print('decoded_info:', decoded_info)
        print('points:', points)
        print('straight_qrcode:', straight_qrcode)

        draw_qr_bounding_box(img, retval, decoded_info, points, straight_qrcode)
    else:
        print('Detection Failed')


def calculate_bounding_box_center(box_points):
    #print('box_points:', box_points)
    x = 0
    y = 0
    n = len(box_points)

    for point in box_points:
        #print('point:', point)
        x = x + point[0]
        y = y + point[1]

    x = int(x / n)
    y = int(y / n)
    #print('x:', x)
    #print('y:', y)

    return (x, y)


def calculate_point_dist_pixels(point0, point1):
    diff_x = point1[0] - point0[0]
    diff_y = point1[1] - point0[1]

    squared_diff_x = diff_x ** 2
    squared_diff_y = diff_y ** 2

    sum_squared_diff = squared_diff_x + squared_diff_y
    
    distance = math.sqrt(sum_squared_diff)
    
    return distance


def calculate_landing_zone_points(points):
    #print('calculate_landing_zone_center')
    # print('points:', points)
        # Aruco
    
    #print('points size:', len(points))
    landing_zone_points = []
    for box in points:
        # print('box:', box)
        if len(box) == 1:
            box = box[0]
            # print('reduced_box_points:', box)
        

        box_center = calculate_bounding_box_center(box)
        landing_zone_points.append(box_center)

    landing_zone_center = calculate_bounding_box_center(landing_zone_points)

    return landing_zone_points, landing_zone_center


def get_img_center(img):
    height, width = img.shape[:2]

    center_x = int(width / 2)
    center_y = int(height / 2)
    img_center = (center_x, center_y)

    return img_center


def draw_img_point(img, point, point_color=(0, 0, 255), point_size=5):
    #print('drawing img')
    img = cv2.circle(img, point, point_size, point_color, -1)
    #print('draw img done')

    return img


def draw_img_center(img):
    img_center = get_img_center(img)

    point_color = (0, 0, 255)  # Red color in BGR format
    point_size = 5

    return draw_img_point(img, img_center, point_color, point_size)


def draw_qr_bounding_box(img, decoded_info, points):
    # print('points:', points)
    annotated_img = cv2.polylines(img, points.astype(int), True, (0, 255, 0), 3)
    
    for s, p in zip(decoded_info, points):
        annotated_img = cv2.putText(annotated_img, s, p[0].astype(int),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2, cv2.LINE_AA)

    return annotated_img


def draw_lz_square(img):
    point_color = (0, 0, 255)

    Width = img.shape[1]
    Height = img.shape[0]

    center_point = (int(Width/2), int(Height/2))

    landing_zone_error_threshold_percentage = 0.1

    square_side = int(min(Width * landing_zone_error_threshold_percentage, Height * landing_zone_error_threshold_percentage))

    # print('Drawing circle')
    # print('center_point:', center_point)
    # print('square_side:', square_side)
    # print('point_color:', point_color)
    img = cv2.circle(img, center_point, square_side, point_color, 2)
    # print('circle done')

    return img

def display_framerate(img, fps):
    # print('display_framerate')
    # print('fps:', fps)
    font = cv2.FONT_HERSHEY_SIMPLEX
    position = (10, 30)  # Top-left corner coordinates

    # Convert the FPS value to string and add it to the frame
    fps_text = "FPS: {:.2f}".format(fps)
    # print('fps_text:', fps_text)
    img = cv2.putText(img, fps_text, position, font, 1, (255, 255, 255), 2)

    return img


def annotate_detector_img(img, retval, decoded_info, points, landing_zone_points, landing_zone_center, fps, use_aruco=False):
    #print('annotate_detector_img start')
    if retval:
        if use_aruco:
            img = cv2.aruco.drawDetectedMarkers(img, points, decoded_info);
        else:
            img = draw_qr_bounding_box(img, decoded_info, points)
        
        for p in landing_zone_points:
            img = draw_img_point(img, p, point_color=(0, 255, 0), point_size=2)
        img = draw_img_point(img, landing_zone_center, point_color=(255, 105, 180), point_size=10)
        
        img = draw_img_center(img)
        img = draw_lz_square(img)
        img = display_framerate(img, fps)

        #print('img:', img)
    #print('annotate_detector_img done')
    
    return img