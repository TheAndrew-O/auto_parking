import cv2
import numpy as np

def find_lines(img, tune_params):
    x_min   = tune_params["x_min"]
    x_max   = tune_params["x_max"]
    y_min   = tune_params["y_min"]
    y_max   = tune_params["y_max"]
    debug_window = [x_min, y_min, x_max, y_max]
    wrk_img = cv2.blur(img, (5, 5))

    if debug_window is None:
        debug_window = [0.0, 0.0, 1.0, 1.0]
    debug_window_px = rect_to_pixels(debug_window, img)

    wrk_img     = cv2.cvtColor(wrk_img, cv2.COLOR_BGR2HSV)
    thresh_min = (tune_params["h_min"], tune_params["s_min"], tune_params["v_min"])
    thresh_max = (tune_params["h_max"], tune_params["s_max"], tune_params["v_max"])
    wrk_img    = cv2.inRange(wrk_img, thresh_min, thresh_max)
    wrk_img = cv2.dilate(wrk_img, None, iterations=2)
    wrk_img = cv2.erode(wrk_img, None, iterations=2)
    tune_img = cv2.bitwise_and(img,img,mask = wrk_img)
    wrk_img = apply_debug_window(wrk_img, debug_window)
    wrk_img = 255 - wrk_img

    params = cv2.SimpleBlobDetector_Params()
    params.minThreshold = 0
    params.maxThreshold = 100
    params.filterByArea = True
    params.minArea = 80
    params.maxArea = 20000
    params.filterByCircularity = True
    params.minCircularity = 0.1
    params.filterByConvexity = True
    params.minConvexity = 0.5
    params.filterByInertia =True
    params.minInertiaRatio = 0.001
    
    # tune_img = cv2.bitwise_and(img,img,mask = wrk_img)
    # out_img = cv2.bitwise_and(img, img, mask = wrk_img)
    # # PARKING LANE DETECTION
    # wrk_img = apply_search_window(wrk_img, search_window)
    # wrk_img = cv2.Canny(wrk_img, 50, 150, apertureSize=3)
    # lines = cv2.HoughLinesP(wrk_img, 1, np.pi/180, 200)
    # if lines is not None:
    #     for line in lines:
    #         x1,y1,x2,y2 = line[0]
    #         cv2.line(out_img, (x1, y1), (x2, y2), (0, 0, 255), 2)
    #         cv2.line(tuning_img, (x1, y1), (x2, y2), (0, 0, 255), 2)
    # out_img = draw_window2(out_img, search_window_px)
    # tuning_img = draw_window2(tuning_img, search_window_px)
    # return lines, out_img, tuning_img

    detector = cv2.SimpleBlobDetector_create(params)
    keypoints = detector.detect(wrk_img)

    size_min_px = tune_params['sz_min'] * wrk_img.shape[1] / 100.0
    size_max_px = tune_params['sz_max'] * wrk_img.shape[1] / 100.0

    keypoints = [k for k in keypoints if k.size > size_min_px and k.size < size_max_px]
    line_color=(0,0,255)
    out_img = cv2.drawKeypoints(img, keypoints, np.array([]), line_color, cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
    out_img = draw_window2(out_img, debug_window_px)
    tune_img = cv2.drawKeypoints(tune_img, keypoints, np.array([]), line_color, cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
    tune_img = draw_window2(tune_img, debug_window_px)
    normal_kp = [normalize(wrk_img, k) for k in keypoints]

    return normal_kp, out_img, tune_img

def apply_debug_window(img, scalar=[0.0, 0.0, 1.0, 1.0]):
    rows = img.shape[0]
    cols = img.shape[1]
    x_min_px = int(cols * scalar[0] / 100)
    y_min_px= int(rows * scalar[1] / 100)
    x_max_px = int(cols * scalar[2] / 100)
    y_max_px = int(rows* scalar[3] / 100)
    mask = np.zeros(img.shape,np.uint8)
    mask[y_min_px:y_max_px,x_min_px:x_max_px] = img[y_min_px:y_max_px,x_min_px:x_max_px]
    return(mask)


#Draw search window: returns the image
def draw_window2(img, rect_px, color=(0,0,255), line=5):
    return cv2.rectangle(img, (rect_px[0],rect_px[1]), (rect_px[2],rect_px[3]), color, line)


def rect_to_pixels(rect, img):
    rows = img.shape[0]
    cols = img.shape[1]
    scale = [cols, rows, cols, rows]
    return [int(a*b/100) for a,b in zip(rect, scale)]

def normalize(img, kp):
    rows = float(img.shape[0])
    cols = float(img.shape[1])
    x_center = 0.5 * cols
    y_center= 0.5 * rows
    x = (kp.pt[0] - x_center)/(x_center)
    y = (kp.pt[1] - y_center)/(y_center)
    return cv2.KeyPoint(x, y, kp.size / img.shape[1])

def create_tuning_window(initial_values):
    cv2.namedWindow("Tuning", 0)
    cv2.createTrackbar("x_min","Tuning",initial_values['x_min'],100,no_op)
    cv2.createTrackbar("x_max","Tuning",initial_values['x_max'],100,no_op)
    cv2.createTrackbar("y_min","Tuning",initial_values['y_min'],100,no_op)
    cv2.createTrackbar("y_max","Tuning",initial_values['y_max'],100,no_op)
    cv2.createTrackbar("h_min","Tuning",initial_values['h_min'],180,no_op)
    cv2.createTrackbar("h_max","Tuning",initial_values['h_max'],180,no_op)
    cv2.createTrackbar("s_min","Tuning",initial_values['s_min'],255,no_op)
    cv2.createTrackbar("s_max","Tuning",initial_values['s_max'],255,no_op)
    cv2.createTrackbar("v_min","Tuning",initial_values['v_min'],255,no_op)
    cv2.createTrackbar("v_max","Tuning",initial_values['v_max'],255,no_op)
    cv2.createTrackbar("sz_min","Tuning",initial_values['sz_min'],100,no_op)
    cv2.createTrackbar("sz_max","Tuning",initial_values['sz_max'],100,no_op)


def get_tune_params():
    tuning_names = ["x_min","x_max","y_min","y_max","h_min","h_max","s_min","s_max","v_min","v_max","sz_min","sz_max"]
    return {key:cv2.getTrackbarPos(key, "Tuning") for key in tuning_names}


def wait_on_gui():
    cv2.waitKey(2)


def no_op(x):
    pass
