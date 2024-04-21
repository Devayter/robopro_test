import numpy as np


BUFFER_SIZE = 1024
NUMBER_OF_MESSAGES = 5
NUMBER_OF_JOINTS = 6
UDP_IP = '127.0.0.1'
UDP_PORT = 8088

DATA_FORMAT = '<Q6d'
ENCODING_FORMAT = 'utf-8'
FILE_NAME = 'robots_position.csv'
GOT_RESPONSE = 'Успешное получение ответа'
LOGGING_FORMAT = '%(asctime)s - %(name)s - %(levelname)s - %(message)s'
NOT_ENOUGHT_THETAS_RECEIVED = (
    'Полученное количество полей "theta" не соответствует ожидаемым'
    'Ожидается - {expected}, получено - {received}'
)
RESULTS_WAS_SAVED = (
    'Расчеты позиций робота были сохранены в файле {file_name}'
)
REQUEST_MESSAGE = b'get'
SEND_REQUEST = 'Отправка запроса'
OUTPUT_FILES_COLUMNS = (
                'Номер принятого сообщения',
                'Позиция в пространстве (X, Y, Z)',
                'Ориентация в пространстве (X-axis, Y-axis, Z-axis)'
)

JOINTS_PARAMS = [
    [0, np.pi/2, 0.21],
    [-0.8, 0, 0.193],
    [-0.598, 0, -0.16],
    [0, np.pi/2, 0.25],
    [0, -np.pi/2, 0.25],
    [0, 0, 0.25]
]
