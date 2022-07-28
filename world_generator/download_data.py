import gdown
from zipfile import ZipFile
import os
import shutil

# MuJoCo training and evaluation environment
# def download(id, name):
#     dataset_url = 'https://drive.google.com/uc?id=' + id
#     dataset_name = "./" + name + ".zip"
#     gdown.download(dataset_url, output = dataset_name, quiet=False)
#     zip_file = ZipFile(dataset_name)
#     zip_file.extractall() # depends on how to zip it
#     zip_file.close()

# download("1hlU2BNlXSieOb4UrQiPJIpoSbivD-6Y_", "husky-ur5-pull")
# download("1sYJorv2KzqA2EvgjIj_ST09ZfKuyp1re", "husky-ur5-push")
# download("1a3NLFWjyF7ClZxxqEYEIgouk57mDMicU", "ur5-pull")
# download("1ri9udFVgK0uw3RPd6l4Ru_doTlWfmu9U", "ur5-push")

# door model
# os.makedirs("door")
dataset_url = 'https://drive.google.com/uc?id=' + "1mLC2WVMUeGNIDR_H97kWdhnOyukk91KS"
dataset_name = "./pullknobs.zip"
gdown.download(dataset_url, output = dataset_name, quiet=True)
zip_file = ZipFile(dataset_name)
zip_file.extractall() # depends on how to zip it
zip_file.close()
# shutil.move("./pullknobs", "./door/pullknobs")