import numpy as np
import os
import time
import onnxruntime as ort

import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory

class InferenceTester(Node):
    def __init__(self):
        super().__init__('inference_tester')

        # onnxruntime
        package_path = get_package_share_directory('inference_tester')
        
        # actor
        actor_path = os.path.join(package_path, 'model', 'actor.onnx')
        self._actor_session = ort.InferenceSession(actor_path, providers=['CPUExecutionProvider'])
        actor_input_names = [input.name for input in self._actor_session.get_inputs()]
        print('Actor input names:', actor_input_names)
        actor_output_names = [output.name for output in self._actor_session.get_outputs()]
        print('Actor output names:', actor_output_names)
        self.actor_input_name = actor_input_names[0]
        self.actor_output_name = actor_output_names[0]
        
        # encoder
        encoder_path = os.path.join(package_path, 'model', 'encoder.onnx')
        self._encoder_session = ort.InferenceSession(encoder_path, providers=['CPUExecutionProvider'])
        encoder_input_names = [input.name for input in self._encoder_session.get_inputs()]
        print('Encoder input names:', encoder_input_names)
        encoder_output_names = [output.name for output in self._encoder_session.get_outputs()]
        print('Encoder output names:', encoder_output_names)
        self.encoder_input_name = encoder_input_names[0]
        self.encoder_output_name = encoder_output_names[0]

        # param
        self.count = 10000
        self.num_actor_input = 22
        self.num_encoder_input = 95

    def run(self):
        actor_obs = np.zeros((self.num_actor_input,), dtype=np.float32)
        encoder_obs = np.zeros((self.num_encoder_input,), dtype=np.float32)
        start = time.time()
        for _ in range(self.count):
            self._actor_session.run([self.actor_output_name], {self.actor_input_name: actor_obs})
            self._encoder_session.run([self.encoder_output_name], {self.encoder_input_name: encoder_obs})
        end = time.time()
        print(f'Count: {self.count}')
        print(f'Elapsed time: {end - start:.3f} s')
        print(f'FPS: {self.count / (end - start):.1f} /s')

def main(args=None):
    rclpy.init(args=args)
    inference_tester = InferenceTester()
    inference_tester.run()
    rclpy.shutdown()
