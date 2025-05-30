#include "sfm.hpp"

/// @brief Конструктор
SFM::SFM(const cv::Mat& camera_matrix, const std::vector<std::vector<cv::Point2f>>& points)
                                                : m_cameraMatrix(camera_matrix), all_pts(points) 
{
    // Проверка согласованности входных данных
    if (all_pts.empty()) throw std::runtime_error("No points provided");
    size_t n_points = all_pts[0].size();
    for (const auto& pts : all_pts) {
        if (pts.size() != n_points) {
            throw std::runtime_error("All point vectors must have the same size");
        }
    }
    
    world_points.resize(n_points, cv::Point3f(0,0,0));
}

// Основной метод запуска обработки
void SFM::run() 
{
    if (all_pts.size() < 2) {
        throw std::runtime_error("At least two frames required");
    }

    // Шаг 1: Инициализация по первым двум кадрам
    initializeFirstTwoFrames();

    // Шаг 2: Добавление остальных кадров
    for (size_t i = 2; i < all_pts.size(); ++i) {
        addNextFrame(i);
    }

    // Шаг 3: Глобальная оптимизация
    bundleAdjustment();
}

// Инициализация по первым двум кадрам
void SFM::initializeFirstTwoFrames() {
    const auto& pts0 = all_pts[0];
    const auto& pts1 = all_pts[1];

    // 1. Находим Essential матрицу
    cv::Mat E, mask;
    E = cv::findEssentialMat(pts0, pts1, m_cameraMatrix, cv::RANSAC, 0.999, 1.0, mask);

    // 2. Восстанавливаем относительную позу
    cv::Mat R, t;
    cv::recoverPose(E, pts0, pts1, m_cameraMatrix, R, t, mask);

    // 3. Сохраняем позы камер
    Rvecs.resize(2);
    tvecs.resize(2);
    Rvecs[0] = cv::Mat::eye(3, 3, CV_64F);
    tvecs[0] = cv::Mat::zeros(3, 1, CV_64F);
    R.convertTo(Rvecs[1], CV_64F);
    t.convertTo(tvecs[1], CV_64F);

    // 4. Триангуляция точек
    triangulatePoints(0, 1);

    initialized = true;
}

// Триангуляция точек между двумя кадрами
void SFM::triangulatePoints(int idx1, int idx2) {
    cv::Mat P1 = m_cameraMatrix * cv::Mat::eye(3, 4, CV_64F);
    cv::Mat P2;
    cv::hconcat(Rvecs[idx2], tvecs[idx2], P2);
    P2 = m_cameraMatrix * P2;

    cv::Mat pts4D;
    cv::triangulatePoints(P1, P2, all_pts[idx1], all_pts[idx2], pts4D);

    // Преобразование в 3D
    for (int i = 0; i < pts4D.cols; ++i) {
        if (world_points[i].z != 0) continue; // Пропускаем уже восстановленные

        cv::Mat x = pts4D.col(i);
        x /= x.at<double>(3);
        world_points[i] = cv::Point3f(x.at<double>(0), 
                                        x.at<double>(1), 
                                        x.at<double>(2));
    }
}

// Добавление нового кадра
void SFM::addNextFrame(size_t frame_idx) {
    // 1. Решаем PnP для текущей камеры
    cv::Mat rvec, tvec;
    std::vector<cv::Point3f> object_pts;
    std::vector<cv::Point2f> image_pts;

    // Собираем известные 3D-2D соответствия
    for (size_t i = 0; i < world_points.size(); ++i) {
        if (world_points[i].z != 0) { // Только восстановленные точки
            object_pts.push_back(world_points[i]);
            image_pts.push_back(all_pts[frame_idx][i]);
        }
    }

    cv::solvePnPRansac(object_pts, image_pts, m_cameraMatrix, cv::noArray(), 
                        rvec, tvec, false, 100, 8.0, 0.99);

    // Сохраняем позу
    Rvecs.push_back(rvec);
    tvecs.push_back(tvec);

    // 2. Триангуляция новых точек с предыдущими кадрами
    for (size_t prev_idx = 0; prev_idx < frame_idx; ++prev_idx) {
        triangulatePoints(prev_idx, frame_idx);
    }
}

// Простейший Bundle Adjustment
void SFM::bundleAdjustment() {
    ceres::Problem problem;
    std::vector<double*> points_params;  // Храним указатели на параметры точек

    // Подготовка данных для Ceres
    std::vector<double*> camera_params;
    for (size_t i = 0; i < Rvecs.size(); ++i) {
        double* params = new double[6]; // [angle_axis (3), translation (3)]
        
        // Преобразование Rvec из OpenCV в angle-axis
        cv::Mat R;
        if (Rvecs[i].rows == 3 && Rvecs[i].cols == 3) {
            cv::Rodrigues(Rvecs[i], R);
        } else {
            cv::Rodrigues(Rvecs[i], R);
        }
        
        cv::Mat angle_axis(3, 1, CV_64F);
        cv::Rodrigues(R, angle_axis);

        memcpy(params, angle_axis.ptr<double>(), 3 * sizeof(double));
        memcpy(params + 3, tvecs[i].ptr<double>(), 3 * sizeof(double));
        
        camera_params.push_back(params);
    }

    // Добавление ограничений для каждой точки
    for (size_t pt_idx = 0; pt_idx < world_points.size(); ++pt_idx) {
        // Пропускаем неинициализированные точки
        if (world_points[pt_idx].z == 0) continue;

        double* point = new double[3];
        point[0] = world_points[pt_idx].x;
        point[1] = world_points[pt_idx].y;
        point[2] = world_points[pt_idx].z;

        points_params.push_back(point);  // Сохраняем указатель

        // Добавляем наблюдения для всех кадров
        for (size_t frame_idx = 0; frame_idx < all_pts.size(); ++frame_idx) {
            const cv::Point2f& obs = all_pts[frame_idx][pt_idx];
            
            ceres::CostFunction* cost_function =
                new ceres::AutoDiffCostFunction<ReprojectionError, 2, 6, 3>(
                    new ReprojectionError(obs, m_cameraMatrix));

            problem.AddResidualBlock(
                cost_function,
                new ceres::HuberLoss(1.0), // Робастное ядро
                camera_params[frame_idx],   // Параметры камеры
                point                      // Параметры точки
            );
        }

        // Фиксируем вершину пирамиды (если нужно)
        if (pt_idx == 4) { // 5-я точка - вершина
            problem.SetParameterBlockConstant(point);
        }
    }

    // Настройки решателя
    ceres::Solver::Options options;
    options.linear_solver_type = ceres::SPARSE_SCHUR;
    options.minimizer_progress_to_stdout = true;
    options.max_num_iterations = 100;
    options.num_threads = 8;

    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
    std::cout << summary.FullReport() << "\n";

    // Обновление параметров после оптимизации
    // 1. Обновление камер
    for (size_t i = 0; i < camera_params.size(); ++i) {
        double* params = camera_params[i];
        
        // Обновление rotation
        cv::Mat angle_axis(3, 1, CV_64F, params);
        cv::Rodrigues(angle_axis, Rvecs[i]);
        
        // Обновление translation
        memcpy(tvecs[i].ptr<double>(), params + 3, 3 * sizeof(double));
        
        delete[] params;
    }

    // 2. Обновление 3D точек
    for (size_t pt_idx = 0; pt_idx < points_params.size(); ++pt_idx) {
        double* point = points_params[pt_idx];
        if (world_points[pt_idx].z == 0) continue;
        
        world_points[pt_idx].x = point[0];
        world_points[pt_idx].y = point[1];
        world_points[pt_idx].z = point[2];
        
        delete[] point;  // Освобождаем память
    }
}

// Фиксация масштаба
void SFM::fixScale() 
{
    if (world_points.empty()) return;
    
    // Предполагаем, что расстояние между 0 и 1 точкой = 1.0
    float dist = cv::norm(world_points[0] - world_points[1]);
    if (dist < 1e-5) return;
    
    float scale = 1.0 / dist;
    for (auto& pt : world_points) pt *= scale;
    for (auto& t : tvecs) t *= scale;
}
