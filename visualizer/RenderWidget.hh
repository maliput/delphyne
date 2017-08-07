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
