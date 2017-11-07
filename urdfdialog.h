#ifndef URDFDIALOG_H
#define URDFDIALOG_H

#include <QDialog>

struct SSimulationThreadCommand
{
    int cmdId;
    std::vector<bool> boolParams;
    std::vector<int> intParams;
    std::vector<float> floatParams;
    std::vector<std::string> stringParams;
};

enum { MAKE_VISIBLE_CMD=0,
       IMPORT_CMD,
       WARNING_MSG_CMD,

 };

namespace Ui {
    class CUrdfDialog;
}

class CUrdfDialog : public QDialog
{
    Q_OBJECT

public:
    explicit CUrdfDialog(QWidget *parent = 0);
    ~CUrdfDialog();

    void refresh();

    void makeVisible(bool visible);
    bool getVisible();

    int dialogMenuItemHandle;

    void reject();

    void addCommand(SSimulationThreadCommand cmd);
    void handleCommands();
    void setSimulationStopped(bool stopped);


private:
    std::vector<SSimulationThreadCommand> _simulThreadCommands;
    static bool hideCollisionLinks;
    static bool hideJoints;
    static bool convexDecompose;
    static bool showConvexDecompositionDlg;
    static bool createVisualIfNone;
    static bool centerModel;
    static bool prepareModel;
    static bool noSelfCollision;
    static bool positionCtrl;
    static bool simulationStopped;

private slots:
    void on_qqImport_clicked();
    void on_qqCollisionLinksHidden_clicked();

    void on_qqJointsHidden_clicked();

    void on_qqConvexDecompose_clicked();

    void on_qqConvexDecomposeDlg_clicked();

    void on_qqCreateVisualLinks_clicked();

    void on_qqCenterModel_clicked();

    void on_qqModelDefinition_clicked();

    void on_qqAlternateMasks_clicked();

    void on_qqPositionCtrl_clicked();

private:
    Ui::CUrdfDialog *ui;
};

#endif // URDFDIALOG_H
