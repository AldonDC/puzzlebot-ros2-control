import cv2

def show_line_debug(roi, cx):
    """Dibuja centro detectado sobre la imagen binaria"""
    vis = cv2.cvtColor(roi, cv2.COLOR_GRAY2BGR) if len(roi.shape) == 2 else roi.copy()
    h, w = vis.shape[:2]
    cv2.line(vis, (cx, 0), (cx, h), (0, 255, 0), 2)
    cv2.circle(vis, (cx, h // 2), 5, (0, 0, 255), -1)
    return vis
