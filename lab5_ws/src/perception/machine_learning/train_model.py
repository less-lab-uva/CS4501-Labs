# Imports
import os
import cv2
import glob
import pickle
import argparse
import numpy as np 
import matplotlib.pyplot as plt
from sklearn.neural_network import MLPClassifier

# Check for user arguments
parser = argparse.ArgumentParser()
parser.add_argument('--epochs',         type=int, default=25)
parser.add_argument('--batch_size',     type=int, default=64)
parser.add_argument('--hidden_layers',  type=str, default="128,128")
parser.add_argument('--h',              type=int, default=48)
parser.add_argument('--w',              type=int, default=48)
args = parser.parse_args()

# Shuffle data
def unison_shuffled_copies(a, b):
    assert len(a) == len(b)
    p = np.random.permutation(len(a))
    return a[p], b[p]

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
    img = cv2.imread(filename)
    img = cv2.resize(img, (48, 48))
    img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    # convert to array
    img = np.asarray(img)
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

    trainX = trainX.reshape((trainX.shape[0], -1))
    testX = testX.reshape((testX.shape[0], -1))

    shuffle_trainX, shuffle_trainY = unison_shuffled_copies(trainX, trainY)
    shuffle_testX, shuffle_testY = unison_shuffled_copies(testX, testY)

    print("Training data shape: " + str(trainX.shape))
    print("Training label shape: " + str(trainY.shape))
    print("Test data shape: " + str(testX.shape))
    print("Test label shape: " + str(testY.shape))

    return shuffle_trainX, shuffle_trainY, shuffle_testX, shuffle_testY

# run the training and save a model
def run_training():
    # load dataset
    trainX, trainY, testX, testY = load_dataset()
    print("Training...")

    # Conver the hidden layer string into a tuple
    layers = tuple(map(int, args.hidden_layers.split(',')))

    model = MLPClassifier(hidden_layer_sizes=layers,
                        activation='relu',
                        solver='adam',
                        learning_rate='adaptive',
                        shuffle=True,
                        verbose=True,
                        early_stopping=False,
                        n_iter_no_change=50,
                        max_iter=args.epochs,
                        batch_size=args.batch_size)
    model.fit(trainX, trainY)
    
    # Get the predictions
    predict_train = model.predict(trainX)
    
    # Convert to from one hot encoding to labels
    predicted_training_labels = np.argmax(predict_train, axis=1)
    actual_training_labels    = np.argmax(trainY, axis=1)

    # Get the accuracy
    training_difference = abs(predicted_training_labels - actual_training_labels)
    training_accuracy = 100.0 - (float(np.count_nonzero(training_difference)) / np.shape(training_difference)[0]) * 100

    print("\n--------------------------------")
    print("Predicted Training Labels: {}".format(predicted_training_labels))
    print("Actual Training Labels: {}".format(actual_training_labels))
    print("Training Accuracy: {}".format(training_accuracy))
    print("--------------------------------\n")

    print("Saving the model...")
    with open('model.pkl','wb') as f:
        pickle.dump(model,f)
    
# run for evaluating a model
def run_testing():
    # load dataset
    trainX, trainY, testX, testY = load_dataset()

    # load model
    print("Loading the model...")
    with open('model.pkl', 'rb') as f:
        model = pickle.load(f)

    # evaluate model on test dataset
    predict_test = model.predict(testX)
    predicted_testing_labels = np.argmax(predict_test, axis=1)

    actual_testing_labels    = np.argmax(testY, axis=1)
    testing_difference = abs(predicted_testing_labels - actual_testing_labels)

    testing_accuracy = 100.0 - (float(np.count_nonzero(testing_difference)) / np.shape(testing_difference)[0]) * 100

    print("\n--------------------------------")
    print("Predicted Testing Labels: {}".format(predicted_testing_labels))
    print("Actual Testing Labels: {}".format(actual_testing_labels))
    print("Testing Accuracy: {}".format(testing_accuracy))
    print("--------------------------------\n")

# load an image and predict the class
def run_single_image():
    classes = get_classes("./training_data/*/")

    # load model
    print("Loading the model...")
    with open('model.pkl', 'rb') as f:
        model = pickle.load(f)

    # For all images in single_prediction
    sample_images = glob.glob("./testing_data/*.jp*")

    print("\nExecuting on a single image: ")
    for img_name in sample_images:
        # Load the image
        img = cv2.imread(img_name)
        img = cv2.resize(img, (48, 48))
        img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        x = img.reshape((1, -1))
        x = x.astype('float32')
        x = x / 255.0
        # predict the class
        prediction = model.predict(x)
        result = np.argmax(prediction, axis=-1)
        print('Single image class (' + img_name + '): ' + str(classes[result[0]]))

# Defining main function
def main():
    print("==============================================")
    print("Training parameters")
    print("Batch size: {}".format(args.batch_size))
    print("Number of epochs: {}".format(args.epochs))
    print("Hidden layers: {}".format(args.hidden_layers))
    print("==============================================")
    print("Training: ")
    run_training()
    print("==============================================")
    print("Testing: ")
    run_testing()
    print("==============================================")
    print("Running Single Image: ")
    run_single_image()
    print("==============================================")

if __name__=="__main__":
    main()
