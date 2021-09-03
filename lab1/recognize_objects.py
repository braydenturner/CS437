import numpy as np
from tflite_runtime.interpreter import Interpreter
from PIL import Image
import io
import picamera
import cv2
import re


class ObjectRecognition:

    CAMERA_WIDTH = 640
    CAMERA_HEIGHT = 480

    def __init__(self, model="/tmp/detect.tflite"):
        self.interpreter = Interpreter("models/detect.tflite")
        self.interpreter.allocate_tensors()
        self.labels = ObjectRecognition.load_labels("models/coco_labels.txt")
        self.camera = picamera.PiCamera(resolution=(ObjectRecognition.CAMERA_WIDTH, ObjectRecognition.CAMERA_HEIGHT), framerate=30)
        self.capture = cv2.VideoCapture(0)
        _, self.input_height, self.input_width, _ = self.interpreter.get_input_details()[0]['shape']

    def __enter__(self):
        return self

    # ...

    def __exit__(self, exc_type, exc_value, traceback):
        self.camera.close()
        self.capture.release()

    @staticmethod
    def load_labels(path):
        """Loads the labels file. Supports files with or without index numbers."""
        with open(path, 'r', encoding='utf-8') as f:
            lines = f.readlines()
            labels = {}
            for row_number, content in enumerate(lines):
                pair = re.split(r'[:\s]+', content.strip(), maxsplit=1)
                if len(pair) == 2 and pair[0].strip().isdigit():
                    labels[int(pair[0])] = pair[1].strip()
                else:
                    labels[row_number] = pair[0].strip()
        return labels

    def set_input_tensor(self, image):
        """Sets the input tensor."""
        tensor_index = self.interpreter.get_input_details()[0]['index']
        input_tensor = self.interpreter.tensor(tensor_index)()[0]
        input_tensor[:, :] = image

    def get_output_tensor(self, index):
        """Returns the output tensor at the given index."""
        output_details = self.interpreter.get_output_details()[index]
        tensor = np.squeeze(self.interpreter.get_tensor(output_details['index']))
        return tensor

    def detect_objects(self, image, threshold):
        """Returns a list of detection results, each a dictionary of object info."""
        self.set_input_tensor(image)
        self.interpreter.invoke()

        # Get all output details
        boxes = self.get_output_tensor(0)
        classes = self.get_output_tensor(1)
        scores = self.get_output_tensor(2)
        count = int(self.get_output_tensor(3))

        results = []
        for i in range(count):
            if scores[i] >= threshold:
                result = {
                    'bounding_box': boxes[i],
                    'class_id': classes[i],
                    'score': scores[i]
                }
                results.append(result)
        return results

    def label_from_class_id(self, classId):
        return self.labels[classId]

    def detect(self):
        ret, frame = self.capture.read()

        #Resize to respect the input_shape
        inp = cv2.resize(frame, (ObjectRecognition.CAMERA_WIDTH , ObjectRecognition.CAMERA_HEIGHT))

        #Convert img to RGB
        rgb = cv2.cvtColor(inp, cv2.COLOR_BGR2RGB)

        results = self.detect_objects(rgb, 0.8)
        return results

    def detect_continuous(self):
        while True:
            ret, frame = self.capture.read()

            #Resize to respect the input_shape
            inp = cv2.resize(frame, (ObjectRecognition.CAMERA_WIDTH , ObjectRecognition.CAMERA_HEIGHT))

            #Convert img to RGB
            rgb = cv2.cvtColor(inp, cv2.COLOR_BGR2RGB)

            results = self.detect_objects(rgb, 0.8)
            objects = [(self.labels[result['class_id']], result['score']) for result in results]
            print(objects)


if __name__ == "__main__":
    obj_recognition = ObjectRecognition()
    try:
        obj_recognition.detect_continuous()
    finally:
        del obj_recognition
