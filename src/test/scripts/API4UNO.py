from roboflow import Roboflow
import cv2
import numpy as np
import tempfile
import os
import requests
import io


rf = Roboflow(api_key="GVcij9mJwc7oVfs78FzN")
project = rf.workspace().project("unocardmodel_v3")
model = project.version(3).model

cap = cv2.VideoCapture(0)
counter = 0
while cap.isOpened():

    ret, frame = cap.read()
    if not ret:
        break
    
    frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB) 

    if counter==100:
        with tempfile.NamedTemporaryFile(suffix=".jpg", delete=False) as temp_file:
            temp_filename = temp_file.name
            cv2.imwrite(temp_filename, frame.squeeze())
            print(model.predict(temp_filename, confidence=40, overlap=30).json())
        counter = 0

    counter+=1
    cv2.imshow("UNO",frame)

    #     with tempfile.NamedTemporaryFile(suffix=".jpg", delete=False) as temp_prediction_file:
    #         temp_prediction_filename = temp_prediction_file.name
    #         model.predict(temp_filename, confidence=40, overlap=30).save(temp_prediction_filename)
    #         prediction = cv2.imread(temp_prediction_filename)

    # ret, buffer = cv2.imencode('.jpg', frame_rgb)
    # # create a file object from the buffer
    # file = io.BytesIO(buffer)

    key=cv2.waitKey(1)
    if key == 27:
        break

# visualize your prediction
# model.predict("your_image.jpg", confidence=40, overlap=30).save("prediction.jpg")

# infer on an image hosted elsewhere
# print(model.predict("URL_OF_YOUR_IMAGE", hosted=True, confidence=40, overlap=30).json())


cap.release()
# plt.close()
cv2.destroyAllWindows()
