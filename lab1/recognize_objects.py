import numpy as np
from tflite_runtime.interpreter import Interpreter
from PIL import Image
import io
import picamera


CAMERA_WIDTH = 640
CAMERA_HEIGHT = 480


class ObjectRecognition:

    def __init__(self, model="/tmp/detect.tflite"):
        self.interpreter = Interpreter("/tmp/detect.tflite")
        self.interpreter.allocate_tensors()
        _, self.input_height, self.input_width, _ = self.interpreter.get_input_details()[0]['shape']

    @staticmethod
    def set_input_tensor(interpreter, image):
        """Sets the input tensor."""
        tensor_index = interpreter.get_input_details()[0]['index']
        input_tensor = interpreter.tensor(tensor_index)()[0]
        input_tensor[:, :] = image

    @staticmethod
    def get_output_tensor(interpreter, index):
        """Returns the output tensor at the given index."""
        output_details = interpreter.get_output_details()[index]
        tensor = np.squeeze(interpreter.get_tensor(output_details['index']))
        return tensor


    def detect_objects(self, image, threshold):
        """Returns a list of detection results, each a dictionary of object info."""
        ObjectRecognition.set_input_tensor(self.interpreter, image)
        self.interpreter.invoke()

        # Get all output details
        boxes = ObjectRecognition.get_output_tensor(self.interpreter, 0)
        classes = ObjectRecognition.get_output_tensor(self.interpreter, 1)
        scores = ObjectRecognition.get_output_tensor(self.interpreter, 2)
        count = int(ObjectRecognition.get_output_tensor(self.interpreter, 3))

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


    def detect(self):
        with picamera.PiCamera(resolution=(CAMERA_WIDTH, CAMERA_HEIGHT), framerate=30) as camera:
            stream = io.BytesIO()
            for _ in camera.capture_continuous(
                    stream, format='jpeg', use_video_port=True):
                image = Image.open(stream).convert('RGB').resize(
                    (self.input_width, self.input_height), Image.ANTIALIAS)
                results = self.detect_objects(self.interpreter, image, 0.4)
                print(results)
                stream.seek(0)
                stream.truncate()

if __name__ == "__main__":
    obj_recognition = ObjectRecognition()
    obj_recognition.detect()
