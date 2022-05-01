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

#include "calibration.h"

#include <easy3d/core/surface_mesh.h>
#include <easy3d/viewer/drawable_triangles.h>
#include <easy3d/viewer/drawable_lines.h>
#include <easy3d/viewer/camera.h>
#include <easy3d/viewer/key_frame_interpolator.h>
#include <easy3d/viewer/manipulated_camera_frame.h>
#include <easy3d/viewer/texture.h>
#include <easy3d/viewer/primitives.h>
#include <easy3d/viewer/setting.h>
#include <easy3d/util/string.h>
#include <easy3d/fileio/resources.h>
#include <easy3d/util/dialogs.h>

#include <3rd_party/glfw/include/GLFW/glfw3.h>	// for the KEYs


using namespace easy3d;


Calibration::Calibration(const std::string& title, const std::string& model_file)
    : Viewer(title)
{
    // Read the point cloud
    if (!add_model(model_file)) {
        std::cerr << "Error: failed model." << std::endl;
        return;
    }
    camera()->setPivotPoint(vec3(0, 0, 0));

    setting::light_position = vec4(0.0f, 0.2f, 1.0f, 0.0f);

    const std::string texture_file = resource::directory() + "/textures/checker.png";
    Texture *tex = Texture::create(texture_file, GL_CLAMP_TO_EDGE);
    TrianglesDrawable *faces = current_model()->triangles_drawable("faces");
    faces->set_texture(tex);
    faces->set_use_texture(true);
    faces->set_distinct_back_color(true);
    faces->set_lighting_two_sides(true);

    SurfaceMesh *mesh = dynamic_cast<SurfaceMesh*>(current_model());
    auto prop = mesh->get_vertex_property<vec3>("v:point");
    std::vector<vec3> end_points;
    for (auto e : mesh->edges()) {
        end_points.push_back(prop[mesh->vertex(e, 0)]);
        end_points.push_back(prop[mesh->vertex(e, 1)]);
    }
    if (!end_points.empty()) {
        LinesDrawable* edges = mesh->add_lines_drawable("edges");
        edges->update_vertex_buffer(end_points);
        edges->set_default_color(vec4(1,1,0,1));
        edges->set_per_vertex_color(false);
        edges->set_impostor_type(LinesDrawable::CYLINDER);
        edges->set_line_width(1);
    }

    // create the X, Y, and Z axes
    const float base = 12.0f;   // the cylinder length, relative to the allowed region
    const float head = 1.0f;   // the cone length, relative to the allowed region
    std::vector<vec3> points, normals, colors;
    opengl::prepare_cylinder(0.1, 10, vec3(0, 0, 0), vec3(base, 0, 0), vec3(1, 0, 0), points, normals, colors);
    opengl::prepare_cylinder(0.1, 10, vec3(0, 0, 0), vec3(0, base, 0), vec3(0, 1, 0), points, normals, colors);
    opengl::prepare_cylinder(0.1, 10, vec3(0, 0, 0), vec3(0, 0, base), vec3(0, 0, 1), points, normals, colors);
    opengl::prepare_cone(0.2, 20, vec3(base, 0, 0), vec3(base + head, 0, 0), vec3(1, 0, 0), points, normals,
                         colors);
    opengl::prepare_cone(0.2, 20, vec3(0, base, 0), vec3(0, base + head, 0), vec3(0, 1, 0), points, normals,
                         colors);
    opengl::prepare_cone(0.2, 20, vec3(0, 0, base), vec3(0, 0, base + head), vec3(0, 0, 1), points, normals,
                         colors);
    TrianglesDrawable *axes = new TrianglesDrawable("corner_axes");
    axes->update_vertex_buffer(points);
    axes->update_normal_buffer(normals);
    axes->update_color_buffer(colors);
    axes->set_per_vertex_color(true);
    axes->set_smooth_shading(true);
    add_drawable(axes);
}


std::string Calibration::usage() const {
    return ("================== Camera Calibration usage ====================\n"
            "\tLeft button: rotate                                           \n"
            "\tRight button: move                                            \n"
            "\tWheel: zoom in/out                                            \n"
            "----------------------------------------------------------------\n"
            "\tKey 's': snapshot (i.e., take a picture)                      \n"
            "\tKey 't': show/hide the virtual camera (if calibration done)   \n"
            "----------------------------------------------------------------\n"
            "\tKey 'space': load 3D-2D correspondences and then calibrate    \n"
            "----------------------------------------------------------------\n");
}


bool Calibration::open() {
    const std::string title("Please choose a file containing the 3D-2D correspondences");
    const std::string default_path(resource::directory() + "/data");
    const std::vector<std::string> &filters = {
            "Correspondences File (*.txt)", "*.txt",
    };

    const std::string &file_name = dialog::open(title, default_path, filters);
    if (file_name.empty())
        return false;

    points_2d_.clear();
    points_3d_.clear();

    FILE *correspondence_file = fopen(file_name.c_str(), "r");
    if (!correspondence_file) {
        LOG(ERROR) << "could not open file: " << file_name;
        return false;
    }

    char line[256];
    double x, y, z, xx, yy;
    while (fgets(line, 256, correspondence_file) != nullptr) {
        if (5 == sscanf(line, "%lf %lf %lf %lf %lf", &x, &y, &z, &xx, &yy)) {
            points_3d_.emplace_back(Vector3D(x, y, z));
            points_2d_.emplace_back(Vector2D(xx, yy));
        }
    }

    // print the points for the students to check if the data has been correctly loaded.
    std::cout << "the correspondences loaded from the file are: " << std::endl;
    for (int i=0; i<points_2d_.size(); ++i) {
        std::cout << "\t" << i << ": (" << points_3d_[i] << ") <-> (" << points_2d_[i] << ")" << std::endl;
    }

    return false;
}


void Calibration::create_cameras_drawable()
{
    camera()->keyFrameInterpolator()->deletePath();

    easy3d::Frame* frame = camera()->frame();
    camera()->keyFrameInterpolator()->addKeyFrame(*frame);
    // update scene bounding box to make sure the path is within the view frustum
    float old_radius = camera()->sceneRadius();
    float radius = distance(camera()->sceneCenter(), camera()->position()) + old_radius * 0.1f;
    camera()->setSceneRadius(std::max(old_radius, radius));

    update();
}


bool Calibration::mouse_drag_event(int x, int y, int dx, int dy, int button, int modifiers) {
    if (modifiers == 0)
        return Viewer::mouse_drag_event(x, y, dx, dy, button, modifiers);
    else
        return false;
}


bool Calibration::mouse_press_event(int x, int y, int button, int modifiers) {
    if (modifiers == GLFW_MOD_SHIFT) {
#if 0
        bool found = false;
        const vec3& p = point_under_pixel(x, y, found);
        if (found) {
            const vec3 p3(std::round(p.x), std::round(p.y), std::round(p.z));
            points_3d_.push_back(p3);
            float scale = dpi_scaling();
            const vec2 p2(std::round(x * scale), std::round(y * scale));
            points_2d_.push_back(p2);
            std::cout << p3 << "\t" << p2 << std::endl;
        }
#endif
        return true;
    }
    else
        return Viewer::mouse_press_event(x, y, button, modifiers);
}


bool Calibration::key_press_event(int key, int modifiers) {
    if (key == GLFW_KEY_SPACE) {
        open();
        if (points_2d_.size() < 6 || points_3d_.size() < 6) {
            LOG(ERROR) << "expecting at least 6 pairs of 3D/2D corresponding points";
            return false;
        }

        double fx, fy, cx, cy, skew;
        Matrix33 R;
        Vector3D t;
        bool success = calibration(points_3d_, points_2d_, fx, fy, cx, cy, skew, R, t);
        if (success) {
            mat3 RR(R(0, 0), R(0, 1), R(0, 2),
                    R(1, 0), R(1, 1), R(1, 2),
                    R(2, 0), R(2, 1), R(2, 2)
            );
            vec3 tt(t.data());
            camera_->set_from_calibration(fx, fy, skew, cx, cy, RR, tt, false);

            create_cameras_drawable();
            return true;
        }
        else {
            LOG(ERROR) << "calibration failed";
            return false;
        }
    }
    else
        return Viewer::key_press_event(key, modifiers);
}