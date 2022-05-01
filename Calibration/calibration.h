/**
 * Copyright (C) 2015 by Liangliang Nan (liangliang.nan@gmail.com)
 * https://3d.bk.tudelft.nl/liangliang/
 *
 * This file is part of Easy3D. If it is useful in your research/work,
 * I would be grateful if you show your appreciation by citing it:
 * ------------------------------------------------------------------
 *      Liangliang Nan.
 *      Easy3D: a lightweight, easy-to-use, and efficient C++
 *      library for processing and rendering 3D data. 2018.
 * ------------------------------------------------------------------
 * Easy3D is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License Version 3
 * as published by the Free Software Foundation.
 *
 * Easy3D is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef CALIBRATION_H
#define CALIBRATION_H

#include <easy3d/viewer/viewer.h>

#include "./vector.h"
#include "./matrix.h"

class Calibration : public easy3d::Viewer
{
public:
    Calibration(const std::string& title, const std::string& model_file);

protected:
    bool open() override;
    std::string usage() const override ;
    bool key_press_event(int key, int modifiers) override;
    bool mouse_press_event(int x, int y, int button, int modifiers) override;
    bool mouse_drag_event(int x, int y, int dx, int dy, int button, int modifiers) override;

    void create_cameras_drawable();

    bool calibration(
            const std::vector<easy3d::Vector3D>& points_3d,
            const std::vector<easy3d::Vector2D>& points_2d,
            double& fx, double& fy,
            double& cx, double& cy,
            double& skew,
            easy3d::Matrix33& R,
            easy3d::Vector3D& t);

private:
    std::vector<easy3d::Vector3D>  points_3d_;
    std::vector<easy3d::Vector2D>  points_2d_;
};


#endif // CALIBRATION_H
