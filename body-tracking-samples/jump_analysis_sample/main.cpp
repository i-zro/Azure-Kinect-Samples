// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#include <array>
#include <iostream>
#include <string>
#include <map>
#include <vector>
#include <fstream> //txt 파일 위해서

#include <k4a/k4a.h>
#include <k4abt.h>

#include <BodyTrackingHelpers.h>
#include <Utilities.h>
#include <Window3dWrapper.h>

#include "JumpEvaluator.h"

using namespace std;

/****************************************
* 30/09/21 코드 : body tracking print*
****************************************/

// 현재 날짜, 시간 받을 ch 배열 전역에 선언
char ch[20];

void print_body_information(uint32_t num, k4abt_body_t body)
{
    std::cout << "관절 번호: " << body.id << std::endl;

    // char 배열인 ch 를 string 형으로 변환
    string str2 = " ";
    str2 = ch;

    // txt 파일 이름
    string file_name = str2 + "/" + "frame" + to_string(num) + ".txt";

    for (int i = 0; i < (int)K4ABT_JOINT_COUNT; i++)
    {

        k4a_float3_t position = body.skeleton.joints[i].position;
        k4a_quaternion_t orientation = body.skeleton.joints[i].orientation;
        k4abt_joint_confidence_level_t confidence_level = body.skeleton.joints[i].confidence_level;

        /****************************************
        * 좌표 txt 파일 작성
        ****************************************/
        printf("Joint[%d]: Position[mm] ( %f, %f, %f ); Orientation ( %f, %f, %f, %f); Confidence Level (%d)  \n",
            i, position.v[0], position.v[1], position.v[2], orientation.v[0], orientation.v[1], orientation.v[2], orientation.v[3], confidence_level);
        std::ofstream writeFile;            // 파일 선언
        writeFile.open(file_name, ios::app);    // 파일 열기
        if (writeFile.is_open())    // 파일이 열렸는지 확인 (추후 정말 시간 남을 때 예외처리 필요)
        {
            writeFile << to_string(i) + "\n";
            writeFile << to_string(position.v[0]) + "\n";
            writeFile << to_string(position.v[1]) + "\n";
            writeFile << to_string(position.v[2]) + "\n";
            writeFile << to_string(orientation.v[0]) + "\n";
            writeFile << to_string(orientation.v[1]) + "\n";
            writeFile << to_string(orientation.v[2]) + "\n";
            writeFile << to_string(orientation.v[3]) + "\n";
            writeFile << to_string(confidence_level) + "\n";
            cout << file_name <<
                " 저장완료" << endl;
            writeFile.close();    // 파일 닫기
        }
    }
}


void PrintAppUsage()
{
    printf("\n");
    printf("\n");
}

// Global State and Key Process Function
bool s_isRunning = true;
bool s_spaceHit = false;

int64_t ProcessKey(void* /*context*/, int key)
{
    // https://www.glfw.org/docs/latest/group__keys.html
    switch (key)
    {
        // Quit
    case GLFW_KEY_ESCAPE:
        s_isRunning = false;
        break;
    case GLFW_KEY_SPACE:
        s_spaceHit = true;
        break;
    case GLFW_KEY_H:
        PrintAppUsage();
        break;
    }
    return 1;
}

int64_t CloseCallback(void* /*context*/)
{
    s_isRunning = false;
    return 1;
}

int main()
{
#pragma region StartingPoint
    PrintAppUsage();

    // 장치 연결 예외 처리
    k4a_device_t device = nullptr;
    VERIFY(k4a_device_open(0, &device), "키넥트 장치 연결 실패!");

    // 장치 연결 예외처리를 통과했다면, 카메라 설정 시작
    k4a_device_configuration_t deviceConfig = K4A_DEVICE_CONFIG_INIT_DISABLE_ALL;
    deviceConfig.depth_mode = K4A_DEPTH_MODE_WFOV_2X2BINNED;
    deviceConfig.color_resolution = K4A_COLOR_RESOLUTION_OFF;
    VERIFY(k4a_device_start_cameras(device, &deviceConfig), 
        "키넥트 카메라가 시작이 안 됨");

    k4a_calibration_t sensorCalibration;
    VERIFY(k4a_device_get_calibration(device, deviceConfig.depth_mode, deviceConfig.color_resolution, &sensorCalibration),
        "깊이 카메라가 안 됨");

    // Body Tracker (k4abt 설정 시작)
    k4abt_tracker_t tracker = nullptr;
    k4abt_tracker_configuration_t tracker_config = K4ABT_TRACKER_CONFIG_DEFAULT;
    VERIFY(k4abt_tracker_create(&sensorCalibration, tracker_config, &tracker), 
        "바디 트래킹 모듈이 실행 안 됨");

    // 3d window controller Initialize 하기
    Window3dWrapper window3d;
    window3d.Create("3D", sensorCalibration);
    window3d.SetCloseCallback(CloseCallback);
    window3d.SetKeyCallback(ProcessKey);
#pragma endregion

    // jump evaluator Initialize 하기
    JumpEvaluator jumpEvaluator;
    
    while (s_isRunning)
    {
        k4a_capture_t sensorCapture = nullptr;
        k4a_wait_result_t getCaptureResult = k4a_device_get_capture(device, &sensorCapture, 0); // timeout_in_ms is set to 0

        if (getCaptureResult == K4A_WAIT_RESULT_SUCCEEDED)
        {
            // timeout_in_ms is set to 0. Return immediately no matter whether the sensorCapture is successfully added
            // to the queue or not.
            k4a_wait_result_t queueCaptureResult = k4abt_tracker_enqueue_capture(tracker, sensorCapture, 0);

            // Release the sensor capture once it is no longer needed.
            k4a_capture_release(sensorCapture);

            if (queueCaptureResult == K4A_WAIT_RESULT_FAILED)
            {
                std::cout << "Error! Add capture to tracker process queue failed!" << std::endl;
                break;
            }
        }
        else if (getCaptureResult != K4A_WAIT_RESULT_TIMEOUT)
        {
            std::cout << "Get depth capture returned error: " << getCaptureResult << std::endl;
            break;
        }

        // Body Tracker 결과 추출 
        k4abt_frame_t bodyFrame = nullptr;
        k4a_wait_result_t popFrameResult = k4abt_tracker_pop_result(tracker, &bodyFrame, 0); 
        // timeout_in_ms is set to 0
        if (popFrameResult == K4A_WAIT_RESULT_SUCCEEDED)
        {
            /************** 
            * 만약 body가 인식되면 여기서 결과 처리
            ***************/

            // capture
            k4a_capture_t originalCapture = k4abt_frame_get_capture(bodyFrame);
#pragma region Jump Analysis
            // Update jump evaluator status
            jumpEvaluator.UpdateStatus(s_spaceHit);
            s_spaceHit = false;

            // Add new body tracking result to the jump evaluator
            const size_t JumpEvaluationBodyIndex = 0; // For simplicity, only run jump evaluation on body 0
            if (k4abt_frame_get_num_bodies(bodyFrame) > 0)
            {
                k4abt_body_t body;
                VERIFY(k4abt_frame_get_body_skeleton(bodyFrame, JumpEvaluationBodyIndex, &body.skeleton), "Get skeleton from body frame failed!");
                body.id = k4abt_frame_get_body_id(bodyFrame, JumpEvaluationBodyIndex);

                uint64_t timestampUsec = k4abt_frame_get_device_timestamp_usec(bodyFrame);
                jumpEvaluator.UpdateData(body, timestampUsec);
            }
#pragma endregion

            // Visualize point cloud
            k4a_image_t depthImage = k4a_capture_get_depth_image(originalCapture);
            window3d.UpdatePointClouds(depthImage);

            // Visualize the skeleton data
            window3d.CleanJointsAndBones();
            uint32_t numBodies = k4abt_frame_get_num_bodies(bodyFrame);

            for (uint32_t i = 0; i < numBodies; i++)
            {
                k4abt_body_t body;

                VERIFY(k4abt_frame_get_body_skeleton(bodyFrame, i, &body.skeleton), "Get skeleton from body frame failed!");
                body.id = k4abt_frame_get_body_id(bodyFrame, i);

                Color color = g_bodyColors[body.id % g_bodyColors.size()];
                color.a = i == JumpEvaluationBodyIndex ? 0.8f : 0.1f;

                window3d.AddBody(body, color);
            }

            k4a_capture_release(originalCapture);
            k4a_image_release(depthImage);
            k4abt_frame_release(bodyFrame);
        }

        window3d.Render();
    }

    std::cout << "Finished jump analysis processing!" << std::endl;

    window3d.Delete();
    k4abt_tracker_shutdown(tracker);
    k4abt_tracker_destroy(tracker);

    k4a_device_stop_cameras(device);
    k4a_device_close(device);

    return 0;
}
