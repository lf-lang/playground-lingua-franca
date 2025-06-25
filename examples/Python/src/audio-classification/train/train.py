import tensorflow as tf
import tflite_model_maker as mm
from tflite_model_maker import audio_classifier
import os

import numpy as np
import matplotlib.pyplot as plt
import seaborn as sns

print(f"TensorFlow Version: {tf.__version__}")
print(f"Model Maker Version: {mm.__version__}")

def show_confusion_matrix(confusion, test_labels):
  """Compute confusion matrix and normalize."""
  confusion_normalized = confusion.astype("float") / confusion.sum(axis=1)
  axis_labels = test_labels
  ax = sns.heatmap(
      confusion_normalized, xticklabels=axis_labels, yticklabels=axis_labels,
      cmap='Blues', annot=True, fmt='.2f', square=True)
  plt.title("Confusion matrix")
  plt.ylabel("True label")
  plt.xlabel("Predicted label")
  plt.savefig('path/to/the/file')

# Define Model Spec
spec = audio_classifier.YamNetSpec(
    keep_yamnet_and_custom_heads=True,
    frame_step= 1 * audio_classifier.YamNetSpec.EXPECTED_WAVEFORM_LENGTH,
    frame_length= 1 * audio_classifier.YamNetSpec.EXPECTED_WAVEFORM_LENGTH)

# Loading the data
dataset_folder = "path/to/the/file"

train_data = audio_classifier.DataLoader.from_folder(
    spec, os.path.join(dataset_folder, 'train'), cache=True)
train_data, validation_data = train_data.split(0.8)

test_data = audio_classifier.DataLoader.from_folder(
    spec, os.path.join(dataset_folder, 'test'), cache=True)

# Start training
batch_size = 128
epochs = 100

print('Training the model')
model = audio_classifier.create(
    train_data,
    spec,
    validation_data,
    batch_size=batch_size,
    epochs=epochs)

# Evaluate model
print('Evaluating the model')
model.evaluate(test_data)

# Understand data
print('Understanding data')
confusion_matrix = model.confusion_matrix(test_data)
show_confusion_matrix(confusion_matrix.numpy(), test_data.index_to_label)

# Export model
models_path = 'path/to/the/file'
print(f'Exporting the TFLite model to {models_path}')
model.export(models_path, tflite_filename='model.tflite')