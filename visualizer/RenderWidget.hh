// Copyright 2017 Open Source Robotics Foundation
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice,
//    this list of conditions and the following disclaimer.
//
// 2. Redistributions in binary form must reproduce the above copyright notice,
//    this list of conditions and the following disclaimer in the documentation
//    and/or other materials provided with the distribution.
//
// 3. Neither the name of the copyright holder nor the names of its contributors
//    may be used to endorse or promote products derived from this software
//    without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#ifndef DELPHYNE_GUI_GUIRENDERWIDGET_HH_
#define DELPHYNE_GUI_GUIRENDERWIDGET_HH_

#include <ignition/gui/Plugin.hh>

#include <ignition/rendering/RenderTypes.hh>
#include <ignition/rendering/RenderingIface.hh>

namespace delphyne {
namespace gui {

class RenderWidget: public ignition::gui::Plugin
{
  //Q_OBJECT

/// \brief Constructor
public: RenderWidget();

/// \brief Destructor
public: virtual ~RenderWidget();

/// \brief Qt paint event.
protected: virtual void paintEvent(QPaintEvent *_e);

protected: virtual void showEvent(QShowEvent *_e);

protected: virtual void resizeEvent(QResizeEvent *_e);

protected: virtual void moveEvent(QMoveEvent *_e);

//// \brief Override paintEngine to stop Qt From trying to draw on top of
/// render window.
/// \return NULL.
protected: virtual QPaintEngine *paintEngine() const;

private: void CreateRenderWindow();

private: QTimer *updateTimer = nullptr;

private: ignition::rendering::RenderWindowPtr renderWindow;
private: ignition::rendering::CameraPtr camera;
};

}  // namespace gui
}  // namespace delphyne

#endif
