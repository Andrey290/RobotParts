#ifndef MOVEMENT_H
#define MOVEMENT_H

#include <vector>
#include <cmath>
#include <algorithm>
#include <memory>

#include "stm32_interface.h"

enum class LidarPointStatus {
    TOO_NEAR,   // < 0.2 м
    VALID,      // 0.2 - 2.5 м
    TOO_FAR,    // > 2.5 м
    INVALID     // Нет данных
};

struct LidarConfig {
    float min_range = 0.2f;      // Минимальная рабочая дистанция (м)
    float max_range = 2.5f;      // Максимальная рабочая дистанция (м)
    float safety_margin = 0.3f;  // Запас безопасности (м)
    float hole_threshold_deg = 5.0f; // Порог для "дыр"
    
    // Сектор, закрытый корпусом (градусы)
    float body_block_start = 150.0f;
    float body_block_end = 210.0f;
    
    // Параметры движения
    float base_speed = 0.3f;     // Базовая скорость (м/с)
    float max_speed = 0.5f;
    float rotation_speed = 0.5f; // Скорость поворота (рад/с)
    
    // Смещение лидара относительно центра (м)
    float lidar_offset_x = 0.1f;  // Вперед от центра
    float lidar_offset_y = 0.0f;  // Вбок от центра
};

class Movement {
public:
    Movement(const LidarConfig& config);
    
    // Основная функция обработки скана
    MotorCommand processScan(const std::vector<float>& ranges, 
                            float current_speed = 0.0f);
    
    // Сброс состояния
    void reset();
    
    // Получить статус точки
    LidarPointStatus getPointStatus(float range) const;
    
    // Получить текущее состояние
    bool isAvoiding() const { return state_ != State::FOLLOWING; }
    bool isEmergencyStop() const { return emergency_stop_; }
    
private:
    enum class State {
        FOLLOWING,      // Следуем траектории
        OBSTACLE_DETECTED, // Обнаружено препятствие
        SIDEWAYS_MOVE,  // Движение вбок
        ROTATING,       // Поворот для поиска пути
        WAITING         // Ожидание очистки пути
    };
    
    LidarConfig config_;
    State state_;
    bool emergency_stop_;
    
    // Для управления состоянием
    int avoidance_counter_;
    float target_angle_;
    float avoidance_direction_; // -1 = влево, 1 = вправо
    
    // Обработка скана
    std::vector<LidarPointStatus> analyzeScan(const std::vector<float>& ranges);
    bool checkForObstacles(const std::vector<LidarPointStatus>& statuses);
    bool checkForLargeHoles(const std::vector<LidarPointStatus>& statuses);
    float findBestDirection(const std::vector<LidarPointStatus>& statuses);
    
    // Логика состояний
    MotorCommand handleFollowing(const std::vector<LidarPointStatus>& statuses);
    MotorCommand handleObstacleDetected(const std::vector<LidarPointStatus>& statuses);
    MotorCommand handleSidewaysMove(const std::vector<LidarPointStatus>& statuses);
    MotorCommand handleRotating(const std::vector<LidarPointStatus>& statuses);
    
    // Вспомогательные функции
    bool isInFrontSector(int angle, float sector_deg = 60.0f) const;
    bool isBodyBlockedAngle(int angle) const;
    float degreeToRadian(float deg) const;
    float radianToDegree(float rad) const;
    
    // Генерация траектории (простая версия)
    MotorCommand generateTrajectoryCommand();
};

#endif