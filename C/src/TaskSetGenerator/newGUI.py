from PyQt5 import QtCore, QtGui, QtWidgets
import sys
import random

class Ui_MainWindow(object):
    

    def setupUi(self, MainWindow):
        scheduler_list = ['NP', 'GEDF_NP', 'GEDF_NP_CI']

        VerticalSize = 1024
        HorizontalSize = 660
        MainWindow.setWindowTitle('Taskset generator for LF schedulers')
        MainWindow.resize(VerticalSize, HorizontalSize)
        MainWindow.setMaximumWidth(VerticalSize)
        MainWindow.setMaximumHeight(HorizontalSize)
        MainWindow.setMinimumWidth(VerticalSize)
        MainWindow.setMinimumHeight(HorizontalSize)
        self.totalTimeUnit = "secs"
        self.executionTimeUnit = "secs"


        self.centralwidget = QtWidgets.QWidget(MainWindow)
        self.centralwidget.setObjectName("centralwidget")

        self.groupBox_scheduler = QtWidgets.QGroupBox(self.centralwidget)
        self.groupBox_scheduler.setGeometry(QtCore.QRect(12, 12, 1000, 100))
        self.groupBox_scheduler.setObjectName("groupBox_scheduler")
        self.groupBox_scheduler.setTitle("Schedulers")

        self.label_scheduler = QtWidgets.QLabel(self.groupBox_scheduler)
        self.label_scheduler.setGeometry(QtCore.QRect(12, 32, 91, 25))
        self.label_scheduler.setObjectName("label_scheduler")
        self.label_scheduler.setText("Scheduler:")
        self.scheduler_NP = QtWidgets.QCheckBox(self.groupBox_scheduler)
        self.scheduler_NP.setGeometry(QtCore.QRect(120, 32, 40, 25))
        self.scheduler_NP.setObjectName('scheduler_NP')
        self.scheduler_NP.setText("NP")
        self.scheduler_GEDF_NP = QtWidgets.QCheckBox(self.groupBox_scheduler)
        self.scheduler_GEDF_NP.setGeometry(QtCore.QRect(180, 32, 100, 25))
        self.scheduler_GEDF_NP.setObjectName('scheduler_GEDF_NP')
        self.scheduler_GEDF_NP.setText("GEDF_NP")
        self.scheduler_GEDF_NP_CI = QtWidgets.QCheckBox(self.groupBox_scheduler)
        self.scheduler_GEDF_NP_CI.setGeometry(QtCore.QRect(280, 32, 120, 25))
        self.scheduler_GEDF_NP_CI.setObjectName('scheduler_GEDF_NP_CI')
        self.scheduler_GEDF_NP_CI.setText("GEDF_NP_CI")

        self.groupBox_generalConfiguration = QtWidgets.QGroupBox(self.centralwidget)              # General Config
        self.groupBox_generalConfiguration.setGeometry(QtCore.QRect(12, 122, 1000, 100))
        self.groupBox_generalConfiguration.setObjectName("groupBox_generalConfiguration")
        self.groupBox_generalConfiguration.setTitle("General Configuration")

        self.label_numOfIterations = QtWidgets.QLabel(self.groupBox_generalConfiguration)
        self.label_numOfIterations.setGeometry(QtCore.QRect(12, 32, 180, 25))
        self.label_numOfIterations.setObjectName("label_numOfIterations")
        self.label_numOfIterations.setText("Number of Iterations:")
        self.spinBox_numOfIterations = QtWidgets.QSpinBox(self.groupBox_generalConfiguration)
        self.spinBox_numOfIterations.setGeometry(QtCore.QRect(200, 32, 55, 25))
        self.spinBox_numOfIterations.setMaximum(100)
        self.spinBox_numOfIterations.setMinimum(1)
        self.spinBox_numOfIterations.setProperty('value', 1)
        self.spinBox_numOfIterations.setObjectName('spinBox_numOfIterations')
        
        self.groupBox_taskConfiguration = QtWidgets.QGroupBox(self.centralwidget)               # Task Config
        self.groupBox_taskConfiguration.setGeometry(QtCore.QRect(12, 232, 1000, 228))
        self.groupBox_taskConfiguration.setObjectName("groupBox_taskConfiguration")
        self.groupBox_taskConfiguration.setTitle("Task Configuration")

        self.tab_task = QtWidgets.QTabWidget(self.groupBox_taskConfiguration)
        self.tab_task.setGeometry(QtCore.QRect(1, 21, 999, 207))
        self.tab_task.setObjectName("tab_task")

        self.scrollArea_task_basic = QtWidgets.QScrollArea(self.tab_task)
        self.scrollArea_task_basic.setWidgetResizable(True)
        self.scrollArea_task_basic.setGeometry(QtCore.QRect(0, 0, 999, 208))
        self.scrollArea_task_basic.setObjectName("scrollArea_task_basic")
        self.scrollArea_task_basic.setVerticalScrollBarPolicy(QtCore.Qt.ScrollBarPolicy.ScrollBarAlwaysOn)
        self.scrollArea_task_basic.setHorizontalScrollBarPolicy(QtCore.Qt.ScrollBarPolicy.ScrollBarAlwaysOff)

        self.scrollAreaWidgetContents_basic = QtWidgets.QWidget()
        self.scrollAreaWidgetContents_basic.setObjectName('scrollAreaWidgetContents_basic')
        
        self.gridLayoutWidget_basic = QtWidgets.QWidget(self.scrollAreaWidgetContents_basic)
        self.gridLayoutWidget_basic.setObjectName("gridLayoutWidget_basic")

        self.gridLayout_basic = QtWidgets.QGridLayout(self.gridLayoutWidget_basic)
        self.gridLayout_basic.setObjectName("gridLayout_basic")

        self.scrollArea_task_basic.setWidget(self.scrollAreaWidgetContents_basic)
        self.scrollAreaWidgetContents_basic.setLayout(self.gridLayout_basic)   
        
        self.tab_task.addTab(self.scrollArea_task_basic, "Basic")

        self.label_periodicity = QtWidgets.QLabel(self.gridLayoutWidget_basic)
        self.label_periodicity.setObjectName("label_periodicity")
        self.label_periodicity.setText("Periodicity:")
        self.label_periodicity.setToolTip("Periodicity of the task: Sporadic/Periodic")
        self.gridLayout_basic.addWidget(self.label_periodicity, 0, 0)
        
        self.comboBox_periodicity = QtWidgets.QComboBox(self.gridLayoutWidget_basic)
        self.comboBox_periodicity.setObjectName("comboBox_periodicity")
        self.comboBox_periodicity.addItems(["sporadic", "periodic"])
        self.comboBox_periodicity.currentIndexChanged.connect(lambda: self.selectionchange(self.comboBox_periodicity, 'periodicity'))
        self.gridLayout_basic.addWidget(self.comboBox_periodicity, 0, 1)
        
        self.label_numOfTasks = QtWidgets.QLabel(self.gridLayoutWidget_basic)
        self.label_numOfTasks.setObjectName("label_numOfTasks")
        self.label_numOfTasks.setText("Number of tasks:")
        self.label_numOfTasks.setToolTip("Number of tasks")
        self.gridLayout_basic.addWidget(self.label_numOfTasks, 1, 0)
        
        self.spinBox_numOfTasks = QtWidgets.QSpinBox(self.gridLayoutWidget_basic)
        self.spinBox_numOfTasks.setObjectName('spinBox_numOfTasks')
        self.spinBox_numOfTasks.setMaximum(100)
        self.spinBox_numOfTasks.setMinimum(1)
        self.spinBox_numOfTasks.setProperty('value', 1)
        self.gridLayout_basic.addWidget(self.spinBox_numOfTasks, 1, 1)

        self.label_totalTime = QtWidgets.QLabel(self.gridLayoutWidget_basic)
        self.label_totalTime.setObjectName("label_totalTime")
        self.label_totalTime.setText("Total time:")
        self.label_totalTime.setToolTip("Total time")
        self.gridLayout_basic.addWidget(self.label_totalTime, 2, 0)

        self.spinBox_totalTime = QtWidgets.QSpinBox(self.gridLayoutWidget_basic)
        self.spinBox_totalTime.setObjectName('spinBox_totalTime')
        self.spinBox_totalTime.setMaximum(10000)
        self.spinBox_totalTime.setMinimum(1)
        self.spinBox_totalTime.setProperty('value', 1)
        self.gridLayout_basic.addWidget(self.spinBox_totalTime, 2, 1)

        self.comboBox_totalTimeUnit = QtWidgets.QComboBox(self.gridLayoutWidget_basic)
        self.comboBox_totalTimeUnit.setObjectName('comboBox_totalTimeUnit')
        self.comboBox_totalTimeUnit.addItems(['secs', 'msecs', 'usecs', 'nsecs'])
        self.comboBox_totalTimeUnit.currentIndexChanged.connect(lambda: self.selectionchange(self.comboBox_totalTimeUnit, 'totalTimeUnit'))
        self.gridLayout_basic.addWidget(self.comboBox_totalTimeUnit, 2, 2)

        self.label_utilization = QtWidgets.QLabel(self.gridLayoutWidget_basic)
        self.label_utilization.setObjectName("label_utilization")
        self.label_utilization.setText("Utilization:")
        self.label_utilization.setToolTip("Utilization")
        self.gridLayout_basic.addWidget(self.label_utilization, 3, 0)

        self.lineEdit_utilization = QtWidgets.QLineEdit(self.gridLayoutWidget_basic)
        self.lineEdit_utilization.setObjectName('lineEdit_utilization')
        self.gridLayout_basic.addWidget(self.lineEdit_utilization, 3, 1)
        
        self.label_period = QtWidgets.QLabel(self.gridLayoutWidget_basic)
        self.label_period.setObjectName("label_period")
        self.label_period.setText("Period:")
        self.label_period.setToolTip("Period for each task")
        self.gridLayout_basic.addWidget(self.label_period, 4, 0)

        self.spinBox_period = QtWidgets.QSpinBox(self.gridLayoutWidget_basic)
        self.spinBox_period.setObjectName('spinBox_period')
        self.spinBox_period.setMaximum(10000)
        self.spinBox_period.setMinimum(1)
        self.spinBox_period.setProperty('value', 1)
        self.gridLayout_basic.addWidget(self.spinBox_period, 4, 1)
        
        self.label_period.hide()
        self.spinBox_period.hide() 

        self.scrollArea_task_dag = QtWidgets.QScrollArea(self.tab_task)
        self.scrollArea_task_dag.setWidgetResizable(True)
        self.scrollArea_task_dag.setGeometry(QtCore.QRect(0, 0, 999, 208))
        self.scrollArea_task_dag.setObjectName("scrollArea_task_dag")
        self.scrollArea_task_dag.setVerticalScrollBarPolicy(QtCore.Qt.ScrollBarPolicy.ScrollBarAlwaysOn)
        self.scrollArea_task_dag.setHorizontalScrollBarPolicy(QtCore.Qt.ScrollBarPolicy.ScrollBarAlwaysOff)

        self.scrollAreaWidgetContents_dag = QtWidgets.QWidget()
        self.scrollAreaWidgetContents_dag.setObjectName('scrollAreaWidgetContents_dag')

        self.gridLayoutWidget_dag = QtWidgets.QWidget(self.scrollAreaWidgetContents_dag)
        self.gridLayoutWidget_dag.setObjectName("gridLayoutWidget_dag")

        self.gridLayout_dag = QtWidgets.QGridLayout(self.gridLayoutWidget_dag)
        self.gridLayout_dag.setObjectName("gridLayout_dag")

        self.scrollArea_task_dag.setWidget(self.scrollAreaWidgetContents_dag)
        self.scrollAreaWidgetContents_dag.setLayout(self.gridLayout_dag)

        self.tab_task.addTab(self.scrollArea_task_dag, "DAG")
        
        self.label_numOfLevel = QtWidgets.QLabel(self.gridLayoutWidget_dag)
        self.label_numOfLevel.setObjectName("label_numOfLevel")
        self.label_numOfLevel.setText("Number of level:")
        self.label_numOfLevel.setToolTip("Number of level")
        self.gridLayout_dag.addWidget(self.label_numOfLevel, 0, 0)

        self.spinBox_numOfLevel = QtWidgets.QSpinBox(self.gridLayoutWidget_dag)
        self.spinBox_numOfLevel.setObjectName('spinBox_numOfLevel')
        self.spinBox_numOfLevel.setMaximum(30)
        self.spinBox_numOfLevel.setMinimum(1)
        self.spinBox_numOfLevel.setProperty('value', 1)
        self.gridLayout_dag.addWidget(self.spinBox_numOfLevel, 0, 1)
    
        self.label_capacityOfOneLevel = QtWidgets.QLabel(self.gridLayoutWidget_dag)
        self.label_capacityOfOneLevel.setObjectName("label_capacityOfOneLevel")
        self.label_capacityOfOneLevel.setText("Maximum number of components in one level:")
        self.label_capacityOfOneLevel.setToolTip("Number of level")
        self.gridLayout_dag.addWidget(self.label_capacityOfOneLevel, 1, 0)

        self.spinBox_capacityOfOneLevel = QtWidgets.QSpinBox(self.gridLayoutWidget_dag)
        self.spinBox_capacityOfOneLevel.setObjectName('spinBox_capacityOfOneLevel')
        self.spinBox_capacityOfOneLevel.setMaximum(30)
        self.spinBox_capacityOfOneLevel.setMinimum(1)
        self.spinBox_capacityOfOneLevel.setProperty('value', 1)
        self.gridLayout_dag.addWidget(self.spinBox_capacityOfOneLevel, 1, 1)
        
        self.label_seed = QtWidgets.QLabel(self.gridLayoutWidget_dag)
        self.label_seed.setObjectName("label_seed")
        self.label_seed.setText("Seed:")
        self.label_seed.setToolTip("Seed value to initialize random function")
        self.gridLayout_dag.addWidget(self.label_seed, 2, 0)

        self.spinBox_seed = QtWidgets.QLineEdit(self.gridLayoutWidget_dag)
        self.spinBox_seed.setObjectName('spinBox_seed')
        self.gridLayout_dag.addWidget(self.spinBox_seed, 2, 1)
        
        self.label_executionTime = QtWidgets.QLabel(self.gridLayoutWidget_dag)
        self.label_executionTime.setObjectName("label_executionTime")
        self.label_executionTime.setText("Execution time of each component:")
        self.label_executionTime.setToolTip("Execution time of one component")
        self.gridLayout_dag.addWidget(self.label_executionTime, 3, 0)

        self.spinBox_executionTime = QtWidgets.QSpinBox(self.gridLayoutWidget_dag)
        self.spinBox_executionTime.setObjectName('spinBox_executionTime')
        self.spinBox_executionTime.setMaximum(10000)
        self.spinBox_executionTime.setMinimum(1)
        self.spinBox_executionTime.setProperty('value', 1)
        self.gridLayout_dag.addWidget(self.spinBox_executionTime, 3, 1)
            
        self.comboBox_executionTimeUnit = QtWidgets.QComboBox(self.gridLayoutWidget_dag)
        self.comboBox_executionTimeUnit.setObjectName('comboBox_executionTimeUnit')
        self.comboBox_executionTimeUnit.addItems(['secs', 'msecs', 'usecs', 'nsecs'])
        self.comboBox_executionTimeUnit.currentIndexChanged.connect(lambda: self.selectionchange(self.comboBox_executionTimeUnit, 'executionTimeUnit'))
        self.gridLayout_dag.addWidget(self.comboBox_executionTimeUnit, 3, 2)

        self.run = QtWidgets.QPushButton(self.centralwidget)
        self.run.setToolTip('Button to run the settings')
        self.run.setGeometry(QtCore.QRect(12, 610, 200, 25))
        self.run.setObjectName("run")
        self.run.setText("Run")

        self.exit = QtWidgets.QPushButton(self.centralwidget)
        self.exit.setToolTip('Exit the framework')
        self.exit.setGeometry(QtCore.QRect(812, 610, 200, 25))
        self.exit.setObjectName('exit')
        self.exit.setText('Exit')

        MainWindow.setCentralWidget(self.centralwidget)


    def selectionchange(self, comboBox, type):
        if type == 'periodicity':
            if comboBox.currentText() == 'sporadic':
                self.label_period.hide()
                self.spinBox_period.hide()
            elif comboBox.currentText() == 'periodic':
                self.label_period.show()
                self.spinBox_period.show()
        elif type == 'totalTimeUnit':
            self.totalTimeUnit = comboBox.currentText()
        elif type == 'executionTimeUnit':
            self.executionTimeUnit = comboBox.currentText()

if __name__ == "__main__":
    app = QtWidgets.QApplication(sys.argv)
    MainWindow = QtWidgets.QMainWindow()
    ui = Ui_MainWindow()
    ui.setupUi(MainWindow)
    MainWindow.statusBar().showMessage('Ready')
    MainWindow.show()
    sys.exit(app.exec_())

    