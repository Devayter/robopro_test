# Тестовое задание для Робопро

## Описание

Приложение представляет собой клиентскую часть системы для работы с манипулятором через UDP протокол. Оно разработано для взаимодействия с сервером, который отправляет пакеты данных с углами поворота сочленений манипулятора.

### Функционал приложения

* Отправка запроса на сервер для получения данных.
* Преобразование полученных пакетов данных в экземпляры dataclass для дальнейшей обработки.
* Расчет прямой задачи кинематики для определения положения робота в пространстве.
* Вывод результатов в файл.

### Использование

1. Клонирование репозитория:
`git clone git@github.com:Devayter/robopro_test.git`
`cd robopro_test`
2. Cоздать и активировать виртуальное окружение:
`python3 -m venv venv`
`source venv/bin/activate`
или для пользователей Windows
`source env/Scripts/activate`
3. Установка зависимостей:
`pip install -r requirements.txt`
4. Запуск сервера:
`gcc main.c`
`./a.out`
5. Запуск приложения:
`python3 main.py`

### Вывод данных

Данные сохраняются в csv формате в базовой директории в файле "robots_positions.csv"
<img width="957" alt="image" src="https://github.com/Devayter/robopro_test/assets/103175986/456d9831-f48d-4889-9c41-3c863f3ea5e9">


## Стек технологий

* Языки программирования: С, Python 3.7+

## Авторы

* [Павел Рябов](https://github.com/Devayter/)
