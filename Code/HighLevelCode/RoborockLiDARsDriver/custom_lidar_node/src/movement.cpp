#include "movement.h"
#include <iostream>

Movement::Movement(const LidarConfig& config)
    : config_(config)
    , state_(State::FOLLOWING)
    , emergency_stop_(false)
    , avoidance_counter_(0)
    , target_angle_(0.0f)
    , avoidance_direction_(1.0f) {
}

void Movement::reset() {
    state_ = State::FOLLOWING;
    emergency_stop_ = false;
    avoidance_counter_ = 0;
}

LidarPointStatus Movement::getPointStatus(float range) const {
    if (std::isnan(range)) {
        return LidarPointStatus::INVALID;
    }
    
    if (range < config_.min_range) {
        return LidarPointStatus::TOO_NEAR;
    } else if (range <= config_.max_range) {
        return LidarPointStatus::VALID;
    } else {
        return LidarPointStatus::TOO_FAR;
    }
}

std::vector<LidarPointStatus> Movement::analyzeScan(const std::vector<float>& ranges) {
    std::vector<LidarPointStatus> statuses(360, LidarPointStatus::INVALID);
    
    for (int i = 0; i < 360; i++) {
        // Игнорируем сектор, закрытый корпусом
        if (isBodyBlockedAngle(i)) {
            statuses[i] = LidarPointStatus::INVALID;
            continue;
        }
        
        statuses[i] = getPointStatus(ranges[i]);
    }
    
    // Заполнение мелких дыр (< 5 градусов)
    for (int i = 0; i < 360; i++) {
        if (statuses[i] == LidarPointStatus::INVALID) {
            // Проверяем окрестность
            bool has_valid_neighbor = false;
            int hole_size = 0;
            
            // Ищем размер дыры
            for (int j = 1; j <= 5; j++) {
                int idx_prev = (i - j + 360) % 360;
                int idx_next = (i + j) % 360;
                
                if (statuses[idx_prev] != LidarPointStatus::INVALID || 
                    statuses[idx_next] != LidarPointStatus::INVALID) {
                    has_valid_neighbor = true;
                }
                
                if (statuses[idx_prev] == LidarPointStatus::INVALID || 
                    statuses[idx_next] == LidarPointStatus::INVALID) {
                    hole_size++;
                }
            }
            
            // Заполняем мелкие дыры статусом TOO_FAR
            if (has_valid_neighbor && hole_size < 5) {
                statuses[i] = LidarPointStatus::TOO_FAR;
            }
        }
    }
    
    return statuses;
}

bool Movement::checkForObstacles(const std::vector<LidarPointStatus>& statuses) {
    // Проверяем передний сектор на препятствия
    for (int angle = 0; angle < 360; angle++) {
        if (isInFrontSector(angle, 45.0f) && !isBodyBlockedAngle(angle)) {
            if (statuses[angle] == LidarPointStatus::TOO_NEAR) {
                return true;
            }
            
            if (statuses[angle] == LidarPointStatus::VALID) {
                // TODO добавить проверку расстояния
                return true;
            }
        }
    }
    return false;
}

bool Movement::checkForLargeHoles(const std::vector<LidarPointStatus>& statuses) {
    int hole_count = 0;
    int max_hole_size = 0;
    int current_hole = 0;
    
    for (int angle = 0; angle < 360; angle++) {
        if (isInFrontSector(angle, 60.0f) && !isBodyBlockedAngle(angle)) {
            if (statuses[angle] == LidarPointStatus::INVALID) {
                current_hole++;
                hole_count++;
            } else {
                if (current_hole > max_hole_size) {
                    max_hole_size = current_hole;
                }
                current_hole = 0;
            }
        }
    }
    
    // Проверяем максимальный размер дыры
    if (current_hole > max_hole_size) {
        max_hole_size = current_hole;
    }
    
    // Если дыра больше порога - считаем опасной
    return (max_hole_size * 1.0f) > config_.hole_threshold_deg;
}

float Movement::findBestDirection(const std::vector<LidarPointStatus>& statuses) {
    // Ищем наиболее свободное направление
    std::vector<float> sector_scores;
    
    // Разбиваем на 8 секторов по 45 градусов
    for (int sector = 0; sector < 8; sector++) {
        float score = 0.0f;
        int count = 0;
        
        for (int angle = sector * 45; angle < (sector + 1) * 45; angle++) {
            int idx = angle % 360;
            
            if (!isBodyBlockedAngle(idx)) {
                switch (statuses[idx]) {
                    case LidarPointStatus::TOO_FAR:
                        score += 2.0f;
                        break;
                    case LidarPointStatus::VALID:
                        // Учитываем расстояние - чем дальше, тем лучше
                        // score += ranges[idx] / config_.max_range;
                        score += 1.0f;
                        break;
                    case LidarPointStatus::TOO_NEAR:
                        score -= 5.0f;
                        break;
                    case LidarPointStatus::INVALID:
                        score -= 3.0f;
                        break;
                }
                count++;
            }
        }
        
        if (count > 0) {
            sector_scores.push_back(score / count);
        } else {
            sector_scores.push_back(-10.0f); // Сектор заблокирован
        }
    }
    
    // Находим лучший сектор
    int best_sector = 0;
    float best_score = sector_scores[0];
    
    for (int i = 1; i < 8; i++) {
        if (sector_scores[i] > best_score) {
            best_score = sector_scores[i];
            best_sector = i;
        }
    }
    
    // Преобразуем сектор в угол (в радианах)
    float best_angle_deg = best_sector * 45.0f;
    return degreeToRadian(best_angle_deg);
}

MotorCommand Movement::processScan(const std::vector<float>& ranges, float current_speed) {
    auto statuses = analyzeScan(ranges);
    
    // Проверка на экстренную остановку
    emergency_stop_ = false;
    for (int angle = 0; angle < 360; angle++) {
        if (isInFrontSector(angle, 30.0f) && !isBodyBlockedAngle(angle)) {
            if (statuses[angle] == LidarPointStatus::TOO_NEAR) {
                emergency_stop_ = true;
                state_ = State::FOLLOWING;
                avoidance_counter_ = 0;
                return MotorCommand(0, 0, 0); // Полная остановка
            }
        }
    }
    
    // Логика состояний
    switch (state_) {
        case State::FOLLOWING:
            return handleFollowing(statuses);
        case State::OBSTACLE_DETECTED:
            return handleObstacleDetected(statuses);
        case State::SIDEWAYS_MOVE:
            return handleSidewaysMove(statuses);
        case State::ROTATING:
            return handleRotating(statuses);
        case State::WAITING:
            // Просто стоп
            avoidance_counter_++;
            if (avoidance_counter_ > 50) { // ~1 секунда
                state_ = State::FOLLOWING;
                avoidance_counter_ = 0;
            }
            return MotorCommand(0, 0, 0);
    }
    
    return MotorCommand(0, 0, 0);
}

MotorCommand Movement::handleFollowing(const std::vector<LidarPointStatus>& statuses) {
    // Проверяем наличие препятствий
    bool obstacle = checkForObstacles(statuses);
    bool large_hole = checkForLargeHoles(statuses);
    
    if (obstacle || large_hole) {
        state_ = State::OBSTACLE_DETECTED;
        avoidance_counter_ = 0;
        
        // Определяем направление для объезда
        target_angle_ = findBestDirection(statuses);
        
        // Решаем, в какую сторону двигаться
        if (target_angle_ > degreeToRadian(180.0f)) {
            avoidance_direction_ = -1.0f; // Влево
        } else {
            avoidance_direction_ = 1.0f; // Вправо
        }
        
        return MotorCommand(0, 0, 0); // Сначала остановимся
    }
    
    // Если препятствий нет - следуем траектории
    return generateTrajectoryCommand();
}

MotorCommand Movement::handleObstacleDetected(const std::vector<LidarPointStatus>& statuses) {
    avoidance_counter_++;
    
    if (avoidance_counter_ < 10) {
        // Пауза перед началом маневра
        return MotorCommand(0, 0, 0);
    }
    
    // Начинаем движение вбок
    state_ = State::SIDEWAYS_MOVE;
    avoidance_counter_ = 0;
    
    return MotorCommand(0, avoidance_direction_ * config_.base_speed * 0.5f, 0);
}

MotorCommand Movement::handleSidewaysMove(const std::vector<LidarPointStatus>& statuses) {
    avoidance_counter_++;
    
    // Двигаемся вбок 1 секунду (~50 циклов при 50Hz)
    if (avoidance_counter_ < 50) {
        return MotorCommand(0, avoidance_direction_ * config_.base_speed * 0.5f, 0);
    }
    
    // Проверяем, свободен ли путь
    bool front_clear = true;
    for (int angle = 0; angle < 360; angle++) {
        if (isInFrontSector(angle, 45.0f) && !isBodyBlockedAngle(angle)) {
            if (statuses[angle] == LidarPointStatus::TOO_NEAR) {
                front_clear = false;
                break;
            }
        }
    }
    
    if (front_clear) {
        // Путь свободен - возвращаемся к траектории
        state_ = State::FOLLOWING;
        avoidance_counter_ = 0;
        return generateTrajectoryCommand();
    } else {
        // Путь все еще заблокирован - пробуем повернуть
        state_ = State::ROTATING;
        avoidance_counter_ = 0;
        return MotorCommand(0, 0, avoidance_direction_ * config_.rotation_speed * 0.5f);
    }
}

MotorCommand Movement::handleRotating(const std::vector<LidarPointStatus>& statuses) {
    avoidance_counter_++;
    
    if (avoidance_counter_ < 30) {
        // Поворачиваем 0.6 секунды
        return MotorCommand(0, 0, avoidance_direction_ * config_.rotation_speed * 0.5f);
    }
    
    // После поворота проверяем снова
    state_ = State::FOLLOWING;
    avoidance_counter_ = 0;
    
    return generateTrajectoryCommand();
}

MotorCommand Movement::generateTrajectoryCommand() {
    // Простая траектория - движение вперед
    //  TODO: алгоритм жука
    
    static float trajectory_phase = 0.0f;
    trajectory_phase += 0.02f; // Обновляем фазу
    
    // Простая синусоидальная траектория
    float vy = 0.1f * std::sin(trajectory_phase);
    
    return MotorCommand(config_.base_speed, vy, 0);
}

bool Movement::isInFrontSector(int angle, float sector_deg) const {
    float half_sector = sector_deg / 2.0f;
    
    // Передний сектор вокруг 0/360 градусов
    float angle_deg = angle;
    
    return (angle_deg <= half_sector) || (angle_deg >= (360.0f - half_sector)) ||
           (std::abs(angle_deg - 360.0f) <= half_sector);
}

bool Movement::isBodyBlockedAngle(int angle) const {
    float angle_deg = angle;
    
    // Проверяем, попадает ли угол в заблокированный сектор
    if (config_.body_block_start < config_.body_block_end) {
        return (angle_deg >= config_.body_block_start && angle_deg <= config_.body_block_end);
    } else {
        // Если сектор пересекает 0/360 градусов
        return (angle_deg >= config_.body_block_start || angle_deg <= config_.body_block_end);
    }
}

float Movement::degreeToRadian(float deg) const {
    return deg * M_PI / 180.0f;
}

float Movement::radianToDegree(float rad) const {
    return rad * 180.0f / M_PI;
}