#include "SnapshotRenderer.h"
#include <raylib.h>
#include <stdexcept>

SnapshotRenderer::SnapshotRenderer(int width, int height)
    : SceneRenderer("Golem2000-Snapshot", width, height, 0)
{}

void SnapshotRenderer::render(const Robot& robot, const std::string& output_path)
{
    // Open a hidden (off-screen) window
    openWindow(/*hidden=*/true);
    if (!isOpen())
        throw std::runtime_error("SnapshotRenderer: failed to open off-screen window");

    // Position the camera to get a good isometric-ish view of the robot
    const Eigen::Vector3d com = robot.centerOfMass();
    camera_distance = 0.8f;  // overridden by Evolver when called via renderRecord
    resetCamera();
    lookAt(static_cast<float>(com.x()),
           static_cast<float>(com.z()),
           static_cast<float>(-com.y()));

    // Render exactly one frame
    BeginDrawing();
        ClearBackground({ 30, 30, 30, 255 });
        BeginMode3D(m_camera);
            drawFloor(floor_grid_slices, floor_grid_spacing);
            drawRobot(robot);
        EndMode3D();

        drawNeuralOverlay(robot, std::vector<double>{});

        // Overlay: robot summary in corner
        DrawText(TextFormat("Robot id=%llu  v=%d  b=%d  n=%d  a=%d",
                            (unsigned long long)robot.id,
                            (int)robot.vertices.size(),
                            (int)robot.bars.size(),
                            (int)robot.neurons.size(),
                            (int)robot.actuators.size()),
                 10, 10, 16, RAYWHITE);
    EndDrawing();

    // Grab the rendered frame from the front buffer and write it out
    Image img = LoadImageFromScreen();
    if (!ExportImage(img, output_path.c_str())) {
        UnloadImage(img);
        closeWindow();
        throw std::runtime_error(
            "SnapshotRenderer: failed to write image to '" + output_path + "'");
    }
    UnloadImage(img);

    closeWindow();
}
