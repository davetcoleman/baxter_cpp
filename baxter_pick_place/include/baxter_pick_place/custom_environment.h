/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013, University of Colorado, Boulder
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Univ of CO, Boulder nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Dave Coleman
   Desc:   Custom environments for running MoveIt!
*/

#include <block_grasp_generator/robot_viz_tools.h> // simple tool for showing grasps

#ifndef BAXTER_PICK_PLACE__CUSTOM_ENVIRONMENT_
#define BAXTER_PICK_PLACE__CUSTOM_ENVIRONMENT_

namespace baxter_pick_place
{

// environment
static const std::string SUPPORT_SURFACE1_NAME = "monitor";
static const std::string SUPPORT_SURFACE2_NAME = "desk";
static const std::string SUPPORT_SURFACE3_NAME = "table";
static const std::string WALL1_NAME = "back_wall";
static const std::string WALL2_NAME = "right_wall";
static const std::string WALL3_NAME = "left_wall";

// table dimensions
static const double TABLE_HEIGHT = 0.83;
static const double TABLE_WIDTH  = 0.85;
static const double TABLE_DEPTH  = 0.44;
static const double TABLE_X = 0.83;
static const double TABLE_Y = 0.15;

// block dimensions
static const double BLOCK_SIZE = 0.04;

void createEnvironment(block_grasp_generator::RobotVizToolsPtr rviz_tools_)
{
  rviz_tools_->cleanupCO(SUPPORT_SURFACE1_NAME);
  rviz_tools_->cleanupCO(SUPPORT_SURFACE2_NAME);
  rviz_tools_->cleanupCO(WALL1_NAME);
  rviz_tools_->cleanupCO(WALL2_NAME);
  rviz_tools_->cleanupCO(WALL3_NAME);

  // --------------------------------------------------------------------------------------------
  // Add objects to scene

  // Walls                          x,     y,     angle,  width, name
  rviz_tools_->publishCollisionWall(-0.55, 0,     0,      2.2,   WALL1_NAME);  // back wall
  rviz_tools_->publishCollisionWall(0.05,  -1.1,  M_PI/2, 2.0,   WALL2_NAME);  // baxter's right
  rviz_tools_->publishCollisionWall(0.05,  1.1,   M_PI/2, 2.0,   WALL3_NAME);  // baxter's left

  // Tables                          x,       y,       angle, width,       height,       depth,       name
  rviz_tools_->publishCollisionTable(0.78,    -0.9,    0,     0.4,         1.4,          0.47,        SUPPORT_SURFACE1_NAME); // computer monitor
  rviz_tools_->publishCollisionTable(0.78,    -0.52,   0,     0.4,         0.7,          0.47,        SUPPORT_SURFACE2_NAME); // my desk
  rviz_tools_->publishCollisionTable(TABLE_X, TABLE_Y, 0,     TABLE_WIDTH, TABLE_HEIGHT, TABLE_DEPTH, SUPPORT_SURFACE3_NAME); // andy table
}

double getTableHeight(double floor_offset)
{
  return TABLE_HEIGHT + floor_offset + BLOCK_SIZE / 2;
}

void getTableWidthRange(double &y_min, double &y_max)
{
  y_min = TABLE_Y - TABLE_WIDTH / 2;
  y_max = TABLE_Y + TABLE_WIDTH / 2;
}

void getTableDepthRange(double &x_min, double &x_max)
{
  x_min = TABLE_X - TABLE_DEPTH / 2;
  x_max = TABLE_X + TABLE_DEPTH / 2;
}

} // namespace

#endif
