#ifndef GUI_RENDER_WIDGET_HH
#define GUI_RENDER_WIDGET_HH

#include <ignition/gui/Plugin.hh>

#include <ignition/rendering/RenderTypes.hh>
#include <ignition/rendering/RenderingIface.hh>

namespace delphyne {
namespace gui {

class GuiRenderWidget: public ignition::gui::Plugin
{
  //Q_OBJECT

/// \brief Constructor
public: GuiRenderWidget();

/// \brief Destructor
public: virtual ~GuiRenderWidget();

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
