# YOLO

This collection of examples show how to process video data in Python and how to invoke a DNN-based object recognition algorithm on the video frames.

# Setup
First, go to the PyTorch website and follow the instructions to install PyTorch: https://pytorch.org/get-started/locally/

IMPORTANT: If running with NVidia GPU, select the correct CUDA version on the installation page. 

Then, install other libraries:

    python3 -m pip install -r requirements.txt

Compile the programs with `lfc`.

# Examples

* [Video.lf](Video.lf): Simple video capture and display. Here, the timing of capturing of frames is controled by a Lingua Franca timer whose period is a parameter of the `WebCam` reactor.
* [VideoAsync.lf](VideoAsync.lf): This is similar except that the frame rate is set on the camera and the `WebCamAsync` reactor blocks on input video frames. This puts the camera in charge of the timing of program execution.
* [YOLOv5_Webcam.lf](YOLOv5_Webcam.lf): This example analyzes each video frame using a pre-trained object-recognition DNN and displays an annotated image. This version uses the `WebCamAsync` reactor from `VideoAsync.l`.
* * [YOLOv5_Webcam_Timer.lf](YOLOv5_Webcam_Timer.lf): This example is similar but use `WebCam` from `Video.lf`, so its timing is driven by a timer rather than by the camera.

