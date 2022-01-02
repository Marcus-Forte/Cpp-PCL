/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2010-2012, Willow Garage, Inc.
 *
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
 *   * Neither the name of Willow Garage, Inc. nor the names of its
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
 *
 */

#include "pcd_view_qt.h"
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include <QApplication>
#include <QButtonGroup>
#include <QEvent>
#include <QFileDialog>
#include <QGroupBox>
#include <QMutexLocker>
#include <QObject>
#include <QRadioButton>
#include <ui_pcd_view_qt.h>

#include <vtkCamera.h>
#include <vtkGenericOpenGLRenderWindow.h>
#include <vtkRenderWindow.h>
#include <vtkRendererCollection.h>

#include <fstream>
#include <iostream>

using namespace pcl;

//////////////////////////////////////////////////////////////////////////////////////////////////////////
PCDViewQt::PCDViewQt()
{
  cloud_present_ = false;
  cloud_modified_ = false;
  play_mode_ = false;
  speed_counter_ = 0;
  speed_value_ = 5;

  // Create a timer
  vis_timer_ = new QTimer(this);
  vis_timer_->start(100); // 100ms

  connect(vis_timer_, SIGNAL(timeout()), this, SLOT(timeoutSlot()));

  ui_ = new Ui::MainWindow;
  ui_->setupUi(this);

  this->setWindowTitle("PCL PCD View");

  // Setup the cloud pointer
  cloud_.reset(new pcl::PointCloud<PointT>);

  // Create the QVTKWidget
#if VTK_MAJOR_VERSION > 8
  auto renderer = vtkSmartPointer<vtkRenderer>::New();
  auto renderWindow = vtkSmartPointer<vtkGenericOpenGLRenderWindow>::New();
  renderWindow->AddRenderer(renderer);
  vis_.reset(new pcl::visualization::PCLVisualizer(renderer, renderWindow, "", false));
#else
  vis_.reset(new pcl::visualization::PCLVisualizer("", false));
#endif // VTK_MAJOR_VERSION > 8
  // setRenderWindowCompat(*(ui_->qvtk_widget), *(vis_->getRenderWindow()));
  // vis_->setupInteractor(getInteractorCompat(*(ui_->qvtk_widget)),
  //                       getRenderWindowCompat(*(ui_->qvtk_widget)));

  // vis_->getInteractorStyle()->setKeyboardModifier(
  //     pcl::visualization::INTERACTOR_KB_MOD_SHIFT);
  // vis_->addCoordinateSystem(1.0);

  auto renderer_ = vtkSmartPointer<vtkRenderer>::New();
  auto renderWindow_ = vtkSmartPointer<vtkGenericOpenGLRenderWindow>::New();
  renderWindow_->AddRenderer(renderer_);
  // vis_dbg.reset(new pcl::visualization::PCLVisualizer(renderer_, renderWindow_, "view", true ));
  vis_dbg.reset(new pcl::visualization::PCLVisualizer("view"));
  

  refreshView();

  // Connect all buttons
  connect(ui_->selectFilesButton,
          SIGNAL(clicked()),
          this,
          SLOT(selectFilesButtonPressed()));
}

void PCDViewQt::selectFilesButtonPressed()
{
  pcd_files_.clear(); // Clear the std::vector
  // pcd_paths_.clear(); // Clear the boost filesystem paths

  QStringList qt_pcd_files = QFileDialog::getOpenFileNames(
      this, "Select one or more PCD files to open", "/home/", "PointClouds (*.pcd)", 0, QFileDialog::DontUseNativeDialog);
  nr_of_frames_ = qt_pcd_files.size();
  PCL_INFO("[pcd_view::selectFilesButtonPressed] : selected %ld files\n",
           nr_of_frames_);

  if (nr_of_frames_ == 0)
  {
    PCL_ERROR("Please select valid pcd files\n");
    cloud_present_ = false;
    return;
  }

  for (int i = 0; i < qt_pcd_files.size(); i++)
  {
    pcd_files_.push_back(qt_pcd_files.at(i).toStdString());
  }

  // TODO automate
  if (pcl::io::loadPCDFile<PointT>(pcd_files_[0], *cloud_) ==
      -1) //* load the file
  {
    PCL_ERROR("[pcd_view_qt::?] : Couldn't read file %s\n");
  }

  PCL_INFO("Lodaded %s (%d points)\n", pcd_files_[0].c_str(), cloud_->size());
  vis_dbg->addPointCloud<PointT>(cloud_, "cloud_");
  // vis_dbg->resetCameraViewpoint("cloud_");
  vis_dbg->spin();

  // if (!vis_->updatePointCloud(cloud_, "cloud_"))
  {
    vis_->addPointCloud<PointT>(cloud_, "cloud_");
    vis_->resetCameraViewpoint("cloud_");
  }
}

void PCDViewQt::timeoutSlot()
{
  // PCL_INFO("timer");
  // std::cout << "time" << std::endl;

  // if (cloud_present_ && cloud_modified_)
  // {
  //   if (pcl::io::loadPCDFile<pcl::PointXYZRGBA>(pcd_files_[current_frame_], *cloud_) ==
  //       -1) //* load the file
  //   {
  //     PCL_ERROR("[PCDVideoPlayer::timeoutSlot] : Couldn't read file %s\n");
  //   }

  //   if (!vis_->updatePointCloud(cloud_, "cloud_"))
  //   {
  //     vis_->addPointCloud(cloud_, "cloud_");
  //     vis_->resetCameraViewpoint("cloud_");
  //   }
  //   cloud_modified_ = false;
  // }

  refreshView();
}

void PCDViewQt::refreshView()
{
#if VTK_MAJOR_VERSION > 8
  ui_->qvtk_widget->renderWindow()->Render();
#else
  ui_->qvtk_widget->update();
#endif // VTK_MAJOR_VERSION > 8
}

void print_usage()
{
  // clang-format off
  PCL_INFO ("Pcd_view V0.1\n");

  // clang-format on
}
