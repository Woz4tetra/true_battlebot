import cv2
import numpy as np
import math

__author__ = "Partha Das"
__license__ = "Free to use and modify as long as I am referred and credited."


class OrientedROISelector:
    """Class providing various functionalities for the selecting polygonal ROI and
    obtaining associated metrics from it.
    Returns a list of dictionaries, where each of the list elements corresponds to a
    ROI. Access the list by accessing the ROIs class variable.
    Each ROI contains the following (The dictionary keys are as labelled here):
    Polygon: A numpy array of points defining the ROI polygon. Can be directly used
             with opencv polygon functions without the need to convert.
    Centroid: A tuple with the coordinate of the centroids of the polygon
    Center: A tuple with the coordinate of the center of the bounding box of the
            polygon
    BoundingBox: The bounding box of the polygon
    ROIRotation: The rotation of the polygon. Only updated when the user explicitly
                 sets the rotation by using the rotation guide enabled by the
                 alternative right and left click. Default is upright, that is zero
                 degrees for the model used.
    """

    def __init__(self, img, windowName=None, autoClose=False):
        self.img = img
        self.__backup = self.img.copy()

        self.ROIs = []
        self.__ROICounter = -1

        self.__polygon = []
        self.__centroid = []
        self.__center = []
        self.__boundingBox = []
        self.__rotation = 0

        self.__POLYSELECTION = 0
        self.__DIRECTION = 1

        self.__mode = self.__POLYSELECTION

        self.windowName = windowName
        self.autoClose = autoClose
        self.__closed = False

        self.__prev = ()

        if windowName != None:
            cv2.setMouseCallback(self.windowName, self.click)
        else:
            self.windowName = "ROI Selection"
            cv2.imshow(self.windowName, self.img)
            cv2.setMouseCallback(self.windowName, self.click)

    def resetCanvas(self, img):
        """Function to reset the canvas with the given image. This resets the current ROI in memory, but leaves the entire ROI list untouched.
        Call this function from the mainloop selectively by listening to specific key strokes.
        """
        self.img = img
        self.__polygon = []
        self.__centroid = []
        self.__center = []
        self.__boundingBox = []
        self.__rotation = 0

        self.__mode = self.__POLYSELECTION
        cv2.imshow(self.windowName, self.img)
        self.__closed = False

    def __updateROI(self):
        """Internal function that updates the ROI list with the current ROI. The rotation is set to 0 since the default assumption is that of
        an upright object. The rotation is not updated till the user explicitly marks the orientation
        """
        if len(self.__polygon) > 0:
            mask = np.zeros(self.img.shape[:2], np.uint8)
            self.__polygon = np.array([self.__polygon], np.int32)
            cv2.fillPoly(mask, self.__polygon, [255] * 3)
            contours, hierarchy = cv2.findContours(
                mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
            )
            if len(contours) > 0:
                contours = contours[0]
                moment = cv2.moments(contours)
                cx = int(moment["m10"] / moment["m00"])
                cy = int(moment["m01"] / moment["m00"])
                rect = cv2.boundingRect(contours)
                self.__centroid = (cx, cy)
                self.__boundingBox = rect
                self.__center = (int(rect[0] + rect[2] / 2), int(rect[1] + rect[3] / 2))
                self.__rotation = 0.0
                tmpDict = {
                    "Polygon": self.__polygon,
                    "Centroid": self.__centroid,
                    "Center": self.__center,
                    "BoundingBox": self.__boundingBox,
                    "ROIRotation": self.__rotation,
                }
                self.ROIs.append(tmpDict)
                self.__polygon = []
            self.__centroid = []
            self.__center = []
            self.__boundingBox = []

    def click(self, event, x, y, flags, param):
        """Main click event for the mouse. Allowed actions:
        Left click: If a ROI is open, that is, it is not enclosed, it adds another point, where the mouse clicked to the polygon
        Right click: If the ROI is open, then it closes the ROI polygon. This was done to make sure that the ROI is closed, since
                     even a pixel of open ROI, while invisible to the human eye, might wreak havoc for algorithms like flood-fill
                     and would need further prepocessing (Trust me, I faced it and this solves a bit of the headaches). Updates
                     the ROI list automatically when the polygon is closed. If the ROI list is empty, then you probably didn't close
                     the polygon. Try right clicking next time.
        Alternative Right click: This is triggered only when the polygon is closed. This starts the orientation mode, recognizable
                     by a line following the mouse from the centroid of the ROI. This mode is used to specify a guide from which
                     the orientation of the ROI is to be calculated.
        Alternative Left click: This is triggered only when the polygon is closed and the orientation mode has started. This
                     finalizes the orientation to face in the direction where the user clicks. So you get a line from which the
                     orientation of the ROI is estimated. The model assumes an upright, right handed, 360 degrees rotational frame.
        """
        if event == cv2.EVENT_FLAG_LBUTTON:
            if self.__mode == self.__POLYSELECTION:
                if self.__closed == True:
                    self.__closed = False
                if len(self.__polygon) > 0:
                    prev = self.__polygon[-1]
                    cv2.line(self.img, (x, y), tuple(prev), [0, 255, 0], 2)
                    cv2.imshow(self.windowName, self.img)
                self.__polygon.append([x, y])
            elif self.__mode == self.__DIRECTION:
                self.__mode = self.__POLYSELECTION
                curX = x
                curY = y
                # Check quadrant
                h = math.hypot(x - self.__prev[0], y - self.__prev[1])
                b = math.hypot(x - x, y - self.__prev[1])
                angle = math.degrees(math.acos(b / h))
                if x >= self.__prev[0]:
                    if y >= self.__prev[1]:
                        angle = 180 - angle

                elif x < self.__prev[0]:
                    if y > self.__prev[1]:
                        angle += 180
                    elif y < self.__prev[1]:
                        angle = 360 - angle
                    elif y == self.__prev[1] and x < self.__prev[0]:
                        angle += 180
                self.__rotation = angle
                self.ROIs[self.__ROICounter]["ROIRotation"] = angle

        elif event == cv2.EVENT_FLAG_RBUTTON:
            if self.__closed == False:
                if len(self.__polygon) >= 3:
                    prev = self.__polygon[0]
                    cur = self.__polygon[-1]
                    cv2.line(self.img, tuple(cur), tuple(prev), [0, 255, 0], 2)
                    cv2.imshow(self.windowName, self.img)
                    self.__closed = True
                    self.__updateROI()
                    self.__ROICounter += 1
            elif self.__closed == True and self.__mode == self.__POLYSELECTION:
                self.__mode = self.__DIRECTION
                self.__backup = self.img.copy()
                self.__prev = self.ROIs[self.__ROICounter]["Centroid"]
                cv2.line(self.img, (x, y), self.__prev, [0, 255, 0], 2)
                cv2.imshow(self.windowName, self.img)
        elif event == cv2.EVENT_MOUSEMOVE:
            if self.__mode == self.__DIRECTION:
                self.img = self.__backup.copy()
                cv2.line(self.img, (x, y), self.__prev, [0, 255, 0], 2)
                cv2.imshow(self.windowName, self.img)
