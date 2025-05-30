#ifndef _FACE_MODEL_POINTS_CALIBRATION_HPP_
#define _FACE_MODEL_POINTS_CALIBRATION_HPP_

#include <array>

class FaceModelPointsCalibration
{
public:
    static constexpr std::array<std::size_t, 4> kPointsIndexes = 
    {
        8,  //подбородок
        12, //угол глаза
        21, //нос
        25  //угол рта
    };

    FaceModelPointsCalibration() = default;

private:
    //
};

#endif //_FACE_MODEL_POINTS_CALIBRATION_HPP_