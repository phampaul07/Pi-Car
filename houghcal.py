import cv2
import numpy as np

def blank(x):
    pass

orig_img = cv2.imread('hallway_test.jpg')

if orig_img is None:
    print("Error: Could not find 'hallway_test.jpg'. Make sure it is in the same folder!")
    exit()

height, width = orig_img.shape[:2]

# Setup OpenCV Window and Sliders
cv2.namedWindow('Hough Calibrator', cv2.WINDOW_NORMAL)
cv2.resizeWindow('Hough Calibrator', 600, 800)


cv2.createTrackbar('Crop Top', 'Hough Calibrator', int(height/2), height-10, blank)
cv2.createTrackbar('Canny Min', 'Hough Calibrator', 50, 255, blank)
cv2.createTrackbar('Canny Max', 'Hough Calibrator', 150, 255, blank)
cv2.createTrackbar('Hough Thresh', 'Hough Calibrator', 80, 200, blank)
cv2.createTrackbar('Min Length', 'Hough Calibrator', 80, 200, blank)
cv2.createTrackbar('Max Gap', 'Hough Calibrator', 30, 100, blank)

while True:
    crop_y = cv2.getTrackbarPos('Crop Top', 'Hough Calibrator')
    c_min = cv2.getTrackbarPos('Canny Min', 'Hough Calibrator')
    c_max = cv2.getTrackbarPos('Canny Max', 'Hough Calibrator')
    h_thresh = cv2.getTrackbarPos('Hough Thresh', 'Hough Calibrator')
    m_len = cv2.getTrackbarPos('Min Length', 'Hough Calibrator')
    m_gap = cv2.getTrackbarPos('Max Gap', 'Hough Calibrator')


    cropped_img = orig_img[crop_y:height, 0:width]


    gray = cv2.cvtColor(cropped_img, cv2.COLOR_BGR2GRAY)
    blurred = cv2.blur(gray, (5, 5))
    edges = cv2.Canny(blurred, c_min, c_max)

    # Apply Hough Lines
    display_img = cropped_img.copy()
    lines = cv2.HoughLinesP(edges, 1, np.pi/180, 
                            threshold=max(1, h_thresh), 
                            minLineLength=m_len, 
                            maxLineGap=m_gap)

    # Draw the lines in red
    if lines is not None:
        for line in lines:
            x1, y1, x2, y2 = line[0]
            cv2.line(display_img, (x1, y1), (x2, y2), (0, 0, 255), 3)


    edges_bgr = cv2.cvtColor(edges, cv2.COLOR_GRAY2BGR)
    stacked = np.vstack((edges_bgr, display_img))

    cv2.imshow('Hough Calibrator', stacked)

    key = cv2.waitKey(1) & 0xFF
    if key == ord('q') or key == 27:
        print(f"cropped_img = img[{crop_y}:, :]")
        print(f"gray = cv2.cvtColor(cropped_img, cv2.COLOR_BGR2GRAY)")
        print(f"blurred = cv2.blur(gray, (5, 5))")
        print(f"edges = cv2.Canny(blurred, {c_min}, {c_max})")
        print(f"lines = cv2.HoughLinesP(edges, 1, np.pi/180, threshold={h_thresh}, minLineLength={m_len}, maxLineGap={m_gap})")
        break

cv2.destroyAllWindows()