import gdown
from zipfile import ZipFile
import os
import shutil

# MuJoCo training and evaluation environment
def download(id, name):
    dataset_url = 'https://drive.google.com/u/1/uc?id=' + id
    dataset_name = "./" + name + ".zip"
    gdown.download(dataset_url, output = dataset_name, quiet=False)
    zip_file = ZipFile(dataset_name)
    zip_file.extractall() # depends on how to zip it
    zip_file.close()

download("1sOhnFgEqJmuSJsuMZ6bliFEgp3BlIeZw", "husky-ur5-pull")
download("1mgjEg11xFxaHh-vFD9Rj3qamtnmSW6ri", "husky-ur5-push")
download("1P8XPVOopCfNcnTN4pvSNKJepROZLhm6L", "ur5-pull")
download("1V6yrJaq_6Nc3ViTt_XPY4c66g0RjK2f8", "ur5-push")

# door model
os.makedirs("./door")
dataset_url = 'https://drive.google.com/u/1/uc?id=' + "1_Gb-QSZaaspL_GYkZBsc0L6mqAOtGQ4v"
dataset_name = "./door/pullknobs.zip"
gdown.download(dataset_url, output = dataset_name, quiet=False)
zip_file = ZipFile(dataset_name)
zip_file.extractall() # depends on how to zip it
zip_file.close()
shutil.move("./pullknobs", "./door/pullknobs")