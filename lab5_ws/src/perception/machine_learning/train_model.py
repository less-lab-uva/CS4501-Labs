# Thanks: https://machinelearningmastery.com/how-to-develop-a-cnn-from-scratch-for-fashion-mnist-clothing-classification/

# model with double the filters for the fashion mnist dataset
import cv2
import glob
import argparse
import numpy as np

from numpy import mean
from numpy import std
from numpy import argmax
from matplotlib import pyplot
from sklearn.model_selection import KFold
from keras.optimizers import Adam
from keras.callbacks import EarlyStopping
from keras.models import Sequential
from keras.models import load_model
from keras.utils import to_categorical
from keras.preprocessing.image import load_img
from keras.preprocessing.image import img_to_array
from keras.layers import Conv2D, Dropout, MaxPooling2D, Dense, Flatten

import os
os.environ['TF_CPP_MIN_LOG_LEVEL'] = '3' 

parser = argparse.ArgumentParser()
parser.add_argument('--epochs', type=int, default=50)
parser.add_argument('--batch_size', type=int, default=64)
parser.add_argument('--h', type=int, default=48)
parser.add_argument('--w', type=int, default=48)
args = parser.parse_args()

# define dnn model (simple)
def define_model(number_classes):
    model = Sequential()
    # model.add(Conv2D(64, (3, 3), padding='same', activation='relu', kernel_initializer='he_uniform', input_shape=(args.h, args.w, 1)))
    # model.add(MaxPooling2D((2, 2)))
    model.add(Flatten())
    model.add(Dense(500, activation='relu', kernel_initializer='he_uniform'))
    # model.add(Dropout(0.2))
    model.add(Dense(500, activation='relu', kernel_initializer='he_uniform'))
    # model.add(Dropout(0.2))
    model.add(Dense(number_classes, activation='softmax'))
    opt = Adam(learning_rate=0.0001)
    # compile model
    model.compile(optimizer=opt, loss='categorical_crossentropy', metrics=['accuracy'])
    return model

# get the classes
def get_classes(dataset):
    # Get the class names from the folder names
    classes = glob.glob(dataset)
    classes.sort()
    for i in range(len(classes)):
        classes[i] = classes[i][:-1]
        pos = classes[i].rfind('/')
        classes[i] = classes[i][pos+1:]
    return classes

# load and prepare the image
def load_image(filename):
    # load the image
    img = load_img(filename, color_mode="grayscale", target_size=(args.h, args.w))
    # convert to array
    img = img_to_array(img)
    # reshape into a single sample with 1 channel
    img = img.reshape(1, args.h, args.w, 1)
    # prepare pixel data
    img = img.astype('float32')
    img = img / 255.0
    return img

# convert a folder to an array
def folder_to_array(file_names, classes):
    x = []
    y = []
    for f in file_names:
        # Create data
        image = load_image(f)
        x.append(image)
        # Create label
        label = []
        # Get the subfolder
        folder_name = f
        pos = folder_name.rfind('/')
        folder_name = folder_name[:pos]
        pos = folder_name.rfind('/')
        folder_name = folder_name[pos+1:]
        # Check if the name is in the subfolder
        for c in classes:
            if c in folder_name:
                label.append(1)
            else:
                label.append(0)
        y.append(label)

    x = np.array(x, dtype='float64')
    y = np.array(y, dtype='int64')

    return x, y

# load the dataset from the folders
def load_dataset():

    # Get the classes
    classes = get_classes("./training_data/*/")
    print("Classes: " + str(classes))

    # Create the training data
    training_files = glob.glob ("./training_data/*/*.jp*") # your image path
    trainX, trainY = folder_to_array(training_files, classes)

    # Create the testing data
    testing_files = glob.glob ("./testing_data/*/*.jp*") # your image path
    testX, testY = folder_to_array(testing_files, classes)

    # Shuffle the data
    idx = np.random.permutation(len(trainX))
    trainX, trainY = trainX[idx], trainY[idx]

    trainX = trainX.reshape((trainX.shape[0], args.h, args.w, 1))
    testX = testX.reshape((testX.shape[0], args.h, args.w, 1))

    print("Training data shape: " + str(trainX.shape))
    print("Training label shape: " + str(trainY.shape))

    print("Test data shape: " + str(testX.shape))
    print("Test label shape: " + str(testY.shape))


    return trainX, trainY, testX, testY

# plot diagnostic learning curves
def summarize_diagnostics(hisotry):
    # plot loss
    pyplot.subplot(111)
    pyplot.title('Classification Accuracy')
    pyplot.plot(hisotry.history['accuracy'], color='blue', label='training accuracy')
    pyplot.plot(hisotry.history['val_accuracy'], color='orange', label='validation accuracy')
    pyplot.legend()
    pyplot.show()

# summarize model performance
def summarize_performance(scores):
    # print summary
    print('Accuracy: mean=%.3f std=%.3f, n=%d' % (mean(scores)*100, std(scores)*100, len(scores)))
    # box and whisker plots of results
    pyplot.boxplot(scores)
    pyplot.show()

# run the training and save a model
def run_training():
    # load dataset
    trainX, trainY, testX, testY = load_dataset()
    # define model
    model = define_model(number_classes=len(testY[0]))
    # Define early stopping
    callback = EarlyStopping(monitor="val_accuracy", patience=250, restore_best_weights=True)
    # fit model
    history = model.fit(trainX, trainY, epochs=args.epochs, batch_size=args.batch_size, verbose=1, validation_split=0.1, shuffle=True, callbacks=[callback])
    # save model
    print(model.summary())
    model.save('marine_model.h5')
    # Display the training data
    summarize_diagnostics(history)

# run for evaluating a model
def run_testing():
    # load dataset
    trainX, trainY, testX, testY = load_dataset()
    # load model
    model = load_model('marine_model.h5')
    # evaluate model on test dataset
    _, acc = model.evaluate(testX, testY, verbose=1)
    print('Test Accuracy: ' + str(acc * 100.0))

# load an image and predict the class
def run_single_image():
    classes = get_classes("./training_data/*/")
    # load model
    model = load_model('marine_model.h5')
    # For all images in single_prediction
    sample_images = glob.glob("./testing_data/*.jp*")
    for img_name in sample_images:
        # Load the image
        image = load_image(img_name)
        # predict the class
        prediction = model.predict(image)
        result = argmax(prediction, axis=-1)
        print('Single image class (' + img_name + '): ' + str(classes[result[0]]))

# Running the code
run_training()
run_testing()
run_single_image()


