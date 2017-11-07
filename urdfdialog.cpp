#include "urdfdialog.h"
#include "ui_urdfdialog.h"
#include "v_repLib.h"
#include "tinyxml2.h"
#include <string>
#include "robot.h"

bool CUrdfDialog::hideCollisionLinks=true;
bool CUrdfDialog::hideJoints=true;
bool CUrdfDialog::convexDecompose=true;
bool CUrdfDialog::showConvexDecompositionDlg=false;
bool CUrdfDialog::createVisualIfNone=true;
bool CUrdfDialog::centerModel=true;
bool CUrdfDialog::prepareModel=true;
bool CUrdfDialog::noSelfCollision=true;
bool CUrdfDialog::positionCtrl=true;
bool CUrdfDialog::simulationStopped=true;

CUrdfDialog::CUrdfDialog(QWidget *parent) :
    QDialog(parent,Qt::Tool),
    ui(new Ui::CUrdfDialog)
{
    ui->setupUi(this);
    refresh();
}

CUrdfDialog::~CUrdfDialog()
{ // Called from the UI thread
    delete ui;
}

void CUrdfDialog::refresh()
{ // Called from the UI thread
    ui->qqAlternateMasks->setEnabled(simulationStopped);
    ui->qqCenterModel->setEnabled(simulationStopped);
    ui->qqCollisionLinksHidden->setEnabled(simulationStopped);
    ui->qqConvexDecompose->setEnabled(simulationStopped);
    ui->qqConvexDecomposeDlg->setEnabled(simulationStopped&&convexDecompose);
    ui->qqCreateVisualLinks->setEnabled(simulationStopped);
    ui->qqImport->setEnabled(simulationStopped);
    ui->qqJointsHidden->setEnabled(simulationStopped);
    ui->qqModelDefinition->setEnabled(simulationStopped);
    ui->qqPositionCtrl->setEnabled(simulationStopped);

    ui->qqAlternateMasks->setChecked(!noSelfCollision);
    ui->qqCenterModel->setChecked(centerModel);
    ui->qqCollisionLinksHidden->setChecked(hideCollisionLinks);
    ui->qqConvexDecompose->setChecked(convexDecompose);
    ui->qqConvexDecomposeDlg->setChecked(showConvexDecompositionDlg);
    ui->qqCreateVisualLinks->setChecked(createVisualIfNone);
    ui->qqJointsHidden->setChecked(hideJoints);
    ui->qqModelDefinition->setChecked(prepareModel);
    ui->qqPositionCtrl->setChecked(positionCtrl);
}

void CUrdfDialog::makeVisible(bool visible)
{ // Called from the UI thread
    setVisible(visible);

    // Reflect the visibility state in the menu bar item:
    SSimulationThreadCommand cmd;
    cmd.cmdId=MAKE_VISIBLE_CMD;
    cmd.boolParams.push_back(visible);
    addCommand(cmd);
}

bool CUrdfDialog::getVisible()
{ // Called from the UI thread
    return(isVisible());
}


void CUrdfDialog::reject()
{ // Called from the UI thread
    // Reflect the visibility state in the menu bar item:
    SSimulationThreadCommand cmd;
    cmd.cmdId=MAKE_VISIBLE_CMD;
    cmd.boolParams.push_back(false);
    addCommand(cmd);

    done(0);
}

void CUrdfDialog::on_qqImport_clicked()
{ // Called from the UI thread
    // handle that command via the main simulation thread:
    SSimulationThreadCommand cmd;
    cmd.cmdId=IMPORT_CMD;
    cmd.boolParams.push_back(hideCollisionLinks);
    cmd.boolParams.push_back(hideJoints);
    cmd.boolParams.push_back(convexDecompose);
    cmd.boolParams.push_back(createVisualIfNone);
    cmd.boolParams.push_back(showConvexDecompositionDlg);
    cmd.boolParams.push_back(centerModel);
    cmd.boolParams.push_back(prepareModel);
    cmd.boolParams.push_back(noSelfCollision);
    cmd.boolParams.push_back(positionCtrl);
    addCommand(cmd);
}

void CUrdfDialog::on_qqCollisionLinksHidden_clicked()
{ // Called from the UI thread
    hideCollisionLinks=!hideCollisionLinks;
    refresh();
}

void CUrdfDialog::on_qqJointsHidden_clicked()
{ // Called from the UI thread
    hideJoints=!hideJoints;
    refresh();
}

void CUrdfDialog::on_qqConvexDecompose_clicked()
{ // Called from the UI thread
    convexDecompose=!convexDecompose;

    if (!convexDecompose)
    {
        SSimulationThreadCommand cmd;
        cmd.cmdId=WARNING_MSG_CMD;
        addCommand(cmd);
    }
    refresh();
}

void CUrdfDialog::on_qqConvexDecomposeDlg_clicked()
{ // Called from the UI thread
    showConvexDecompositionDlg=!showConvexDecompositionDlg;
    refresh();
}

void CUrdfDialog::on_qqCreateVisualLinks_clicked()
{ // Called from the UI thread
    createVisualIfNone=!createVisualIfNone;
    refresh();
}

void CUrdfDialog::on_qqCenterModel_clicked()
{ // Called from the UI thread
    centerModel=!centerModel;
    refresh();
}

void CUrdfDialog::on_qqModelDefinition_clicked()
{ // Called from the UI thread
    prepareModel=!prepareModel;
    refresh();
}

void CUrdfDialog::on_qqAlternateMasks_clicked()
{ // Called from the UI thread
    noSelfCollision=!noSelfCollision;
    refresh();
}

void CUrdfDialog::on_qqPositionCtrl_clicked()
{ // Called from the UI thread
    positionCtrl=!positionCtrl;
    refresh();
}

void CUrdfDialog::setSimulationStopped(bool stopped)
{
    simulationStopped=stopped;
}

void CUrdfDialog::addCommand(SSimulationThreadCommand cmd)
{
    _simulThreadCommands.push_back(cmd);
}

void CUrdfDialog::handleCommands()
{ // Called from the main SIM thread
    for (int i=0;i<int(_simulThreadCommands.size());i++)
    {
        SSimulationThreadCommand cmd=_simulThreadCommands[i];
        if (cmd.cmdId==MAKE_VISIBLE_CMD)
            simSetModuleMenuItemState(dialogMenuItemHandle,(cmd.boolParams[0]?3:1),"URDF import...");
        if (cmd.cmdId==IMPORT_CMD)
        {
            simChar* pathAndFile=simFileDialog(sim_filedlg_type_load,"URDF PLUGIN LOADER","","","URDF Files","urdf");
            if (pathAndFile!=NULL)
            {
                std::string fileURDF(pathAndFile);
                simReleaseBuffer(pathAndFile);
                // The full path and filename is in pathAndFile!
                // Here we can import from that file
                robot Robot(fileURDF,cmd.boolParams[0],cmd.boolParams[1],cmd.boolParams[2],cmd.boolParams[3],cmd.boolParams[4],cmd.boolParams[5],cmd.boolParams[6],cmd.boolParams[7],cmd.boolParams[8]);
            }
        }
        if (cmd.cmdId==WARNING_MSG_CMD)
            simMsgBox(sim_msgbox_type_warning,sim_msgbox_buttons_ok,"WARNING","It is highly recommended to perform a convex decomposition: convex collidables are more stable and faster than random collidables.");
    }
    _simulThreadCommands.clear();
}
