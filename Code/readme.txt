Здесь вы можете найти как код верхнего уровня RPi 4 так и код нижнего
уровня STM32F407G-DISC1 

.
├── HighLevelCode
│   └── RoborockLiDARsDriver
│       ├── CustomLidarNode
│       └── OfficialSoftware
└── LowLevelCode
    ├── Arduino
    │   └── sketch_aug18a.ino
    ├── MotorsControllerESP
    │   └── hello_world
    └── MotorsControllerSTM
        ├── Core
        └── project1.ioc

Остались также наброски кода для реализаций на ESP и Arduino а также
официальная нода лидара под humble. Моя для jazzy.

В MotorControllerSTM32 лежит исходный код и .ioc файл.

Я работал в HAL и CubeMX. Настроено управление ШИМ, считываются данные
с энкодера и с датчика тока. Вывод по поднятию флага. В качестве
таймера главного цикла выступает базовый таймер TIM1.
