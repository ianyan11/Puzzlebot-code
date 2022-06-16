import os
import numpy as np
import tensorflow as tf
from google import protobuf
from tensorflow.python.compiler.tensorrt import trt_convert as trt

print("Tensorflow version: ", tf.version.VERSION)
print("Protobuf version:", protobuf.__version__)
print("TensorRT version: ")
print(os.system("dpkg -l | grep TensorRT"))

gpu_devices = tf.config.experimental.list_physical_devices('GPU')
tf.config.experimental.set_memory_growth(gpu_devices[0], True)
tf.config.experimental.set_virtual_device_configuration(
            gpu_devices[0],
            [tf.config.experimental.VirtualDeviceConfiguration(
               memory_limit=1024)]) ## Crucial value, set lower than available GPU memory (note that Jetson shares GPU memory with CPU), should be 2048

conversion_params = trt.DEFAULT_TRT_CONVERSION_PARAMS
conversion_params = conversion_params._replace(max_workspace_size_bytes=(1<<30)) 
conversion_params = conversion_params._replace(precision_mode="FP16")
conversion_params = conversion_params._replace(maximum_cached_engines=10)
conversion_params = conversion_params._replace(use_calibration=True)
converter = trt.TrtGraphConverterV2(
    input_saved_model_dir="./trained_model/saved_model",
    conversion_params=conversion_params)
converter.convert() 

batch_size = 1
def input_fn():
    # Substitute with your input size
    Inp1 = np.random.normal(size=(batch_size, 1024, 1024, 3)).astype(np.uint8) 
    yield (Inp1, )
converter.build(input_fn=input_fn)

converter.save("./trained_model/saved_model_compressed_int8")