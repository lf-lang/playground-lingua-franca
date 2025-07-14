# Dataset and Training

## Dataset Details
The training dataset for the Audio Classification model comprises two distinct classes:

- **Emergency**: This class encompasses 850 .wav files featuring sirens from emergency vehicles like Ambulances, Firetrucks, and Police units. These files were meticulously extracted from the [Emergency Vehicle Siren Sounds][EVSS] and [SireNNet][SireNNet] Dataset.

- **Other**: Comprising 800 .wav files, this class includes a diverse range of sounds manually extracted from the [ESC-50 Dataset][ESC50]. These encompass various categories such as Animals, Natural Soundscapes & Water Sounds, Human Non-Speech Sounds, Interior/Domestic Sounds, and Urban Sounds (excluding sirens).

You can acces the _.zip_ file containing the dataset a this [link][Drive]

The internal directory structure is the following:
```bash
├── dataset
│   ├── test
│   │   ├── emergency
│   │   └──other
│   ├── test
│   │   ├── emergency
└── └── └── other
```

## Training

Once you have downloaded the dataset, you have the option to either train your own machine learning model or utilize the `train.py` file to perform Transfer Learning using a pre-trained audio classification model like `YAMNet`.

Before initiating the transfer learning task, ensure that you have the following Python libraries installed:
```bash
tensorflow
tflite_model_maker
numpy
matplotlib
seaborn
```

Additionally, inside the `train.py` script, make sure to fill in all the placeholders for paths (`path/to/the/file`) as required.


[ESC50]: https://github.com/karolpiczak/ESC-50
[SireNNet]: https://data.mendeley.com/datasets/j4ydzzv4kb/1
[EVSS]: https://www.kaggle.com/datasets/vishnu0399/emergency-vehicle-siren-sounds
[Drive]: https://drive.google.com/file/d/1iLDItoe9v7zL1AIz2bP2OVVRcN2oTziD/view?usp=drive_link