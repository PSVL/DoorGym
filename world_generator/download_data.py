import gdown
from zipfile import ZipFile
import os
import shutil

# MuJoCo training and evaluation environment
dataset_url = 'https://drive.google.com/u/1/uc?id=' + "1000pGwD9iM51FSeLDRfZi0LvdGGtqhdn"
dataset_name = "./" + "MuJoCo_environment"+ ".zip"
gdown.download(dataset_url, output = dataset_name, quiet=True)
zip_file = ZipFile(dataset_name)
zip_file.extractall() # depends on how to zip it
zip_file.close()

# door model
os.makedirs("door")
dataset_url = 'https://drive.google.com/u/1/uc?id=' + "1_Gb-QSZaaspL_GYkZBsc0L6mqAOtGQ4v"
dataset_name = "./door/pullknobs.zip"
gdown.download(dataset_url, output = dataset_name, quiet=False)
zip_file = ZipFile(dataset_name)
zip_file.extractall() # depends on how to zip it
zip_file.close()
shutil.move("./pullknobs", "./door/pullknobs")