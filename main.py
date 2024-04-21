import csv
import logging
import socket
import struct
from copy import deepcopy
from typing import List


from constants import (
    BUFFER_SIZE, DATA_FORMAT, ENCODING_FORMAT, FILE_NAME, GOT_RESPONSE,
    JOINTS_PARAMS, LOGGING_FORMAT, NUMBER_OF_MESSAGES, NUMBER_OF_JOINTS,
    NOT_ENOUGHT_THETAS_RECEIVED, OUTPUT_FILES_COLUMNS, RESULTS_WAS_SAVED,
    REQUEST_MESSAGE, SEND_REQUEST, UDP_IP, UDP_PORT
)
from models import DHKinematics


def get_models(sock: socket):
    """
    Функция получения списка моделей DHKinematics для дальнейшего расчета.
    """
    dh_models = []
    for i in range(NUMBER_OF_MESSAGES):
        data, addr = sock.recvfrom(BUFFER_SIZE)
        msg = struct.unpack(DATA_FORMAT, data)
        thetas = msg[1:]
        if len(thetas) != NUMBER_OF_JOINTS:
            raise ValueError(
                NOT_ENOUGHT_THETAS_RECEIVED.format(
                    expected=NUMBER_OF_JOINTS, received=len(thetas)
                )
            )
        joints_params = deepcopy(JOINTS_PARAMS)
        for i, theta in enumerate(thetas):
            joints_params[i].append(theta)
        dh_models.append(DHKinematics(joints_params))
    return dh_models


def get_results(dh_models: List[DHKinematics]):
    """
    Функция расчета финальной позиции и ориентации робота.
    """
    results = []
    for i, dh_model in enumerate(dh_models, 1):
        T = dh_model.forward_kinematics()
        x, y, z = T[:3, 3]
        P = [x, y, z]
        R = T[:3, :3]
        results.append((
            i,
            f'({P[0]:.4f}, {P[1]:.4f}, {P[2]:.4f})',
            f'X: ({R[0, 0]:.4f}, {R[1, 0]:.4f}, {R[2, 0]:.4f}), '
            f'Y: ({R[0, 1]:.4f}, {R[1, 1]:.4f}, {R[2, 1]:.4f}), '
            f'Z: ({R[0, 2]:.4f}, {R[1, 2]:.4f}, {R[2, 2]:.4f})'
        ))
    return results


def write_results_to_file(results):
    """
    Функция вывода результатов в файл.
    """
    with open(FILE_NAME, 'w', encoding=ENCODING_FORMAT) as f:
        writer = csv.writer(f, dialect=csv.unix_dialect)
        writer.writerows([OUTPUT_FILES_COLUMNS, *results])


def main():
    """
    Основная функция приложения для работы с манипулятором через UDP протокол.
    Функция выполняет следующие действия:
    1. Настройка логгирования.
    2. Создание UDP сокета и отправка запроса на сервер для получения данных.
    3. Получение и преобразование данных в модели Денавита-Хартенберга.
    4. Рассчет прямой задачи кинематики.
    5. Запись результатов расчетов в файл.
    """
    logging.basicConfig(format=LOGGING_FORMAT, level=logging.INFO)
    logger = logging.getLogger(__name__)
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    logger.info(SEND_REQUEST)
    sock.sendto(REQUEST_MESSAGE, (UDP_IP, UDP_PORT))
    dh_models = get_models(sock)
    logger.info(GOT_RESPONSE)
    sock.close()
    write_results_to_file(get_results(dh_models))
    logger.info(RESULTS_WAS_SAVED.format(file_name=FILE_NAME))


if __name__ == '__main__':
    main()
