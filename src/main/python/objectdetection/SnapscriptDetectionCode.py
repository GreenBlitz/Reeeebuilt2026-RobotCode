import cv2
import numpy as np

lower_yellow = np.array([20, 100, 100])
upper_yellow = np.array([30, 255, 255]) #wrong values impact the accuracy a lot. PLS NEXT PROJECT MAKE GOOD CALIBARATION CODE IN PYTHON

COLUMNS = 25

def count_yellow_in_cell(mask, x_start, x_end, y_start, y_end):
    cell_mask = mask[y_start:y_end, x_start:x_end]
    return np.sum(cell_mask)

def runPipeline(image, llrobot):
    height, width = image.shape[:2]

    start = np.float32([[0, 0], [width, 0], [0, height], [width, height]])
    top_y = (start[0][1] + start[1][1]) / 2
    bottom_y = (start[2][1] + start[3][1]) / 2
    xfactor = bottom_y - top_y
    yfactor = -xfactor

    dsize = (int(width), int(height + xfactor))
    end = np.float32([
        [0, 0],
        [width, 0],
        [-yfactor/2, height + xfactor],
        [width + yfactor/2, height + xfactor]
    ])

    matrix = cv2.getPerspectiveTransform(start, end)
    inv_matrix = np.linalg.inv(matrix)
    warped = cv2.warpPerspective(image, matrix, dsize, flags=cv2.INTER_LINEAR)

    heightAfter, widthAfter = warped.shape[:2]

    hsv_img = cv2.cvtColor(warped, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv_img, lower_yellow, upper_yellow)

    rows = round(heightAfter / (widthAfter / COLUMNS))
    cell_w = int(widthAfter / COLUMNS)
    cell_h = int(heightAfter / rows)

    most_concentrated = -1
    most_x, most_y = -1, -1

    for i in range(rows):
        for j in range(COLUMNS):
            x_start = j * cell_w
            x_end = (j + 1) * cell_w
            y_start = i * cell_h
            y_end = (i + 1) * cell_h

            avg = count_yellow_in_cell(mask, x_start, x_end, y_start, y_end) / (cell_h * cell_w)
            percent = avg * 100 / 255

            if percent > most_concentrated:
                most_concentrated = percent
                most_x = j
                most_y = i

    middle_x = most_x * cell_w + cell_w // 2
    middle_y = most_y * cell_h + cell_h // 2

    pt = np.array([[middle_x], [middle_y], [1]])
    orig_pt = inv_matrix @ pt
    orig_pt = orig_pt.flatten()

    orig_x = int(orig_pt[0] / orig_pt[2])
    orig_y = int(orig_pt[1] / orig_pt[2])

    nx = (orig_x - (width / 2)) / (width / 2)
    ny = (orig_y - (height / 2)) / (height / 2)

    contour = np.array([
        [[orig_x - cell_w, orig_y - cell_h]],
        [[orig_x + cell_w, orig_y - cell_h]],
        [[orig_x + cell_w, orig_y + cell_h]],
        [[orig_x - cell_w, orig_y + cell_h]]
    ])

    x, y, w, h = cv2.boundingRect(contour)

    cv2.rectangle(image, (x, y), (x + w, y + h), (0, 255, 255), 2)

    llpython = [1, nx, ny, orig_x, orig_y, most_concentrated, 0, 0]

    print(f"Grid cell: ({most_x}, {most_y}) | Pixel: ({orig_x}, {orig_y}) | nx={nx:.3f}, ny={ny:.3f}")

    return contour, image, llpython