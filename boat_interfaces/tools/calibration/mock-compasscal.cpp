#include <iostream>
#include <limits>

using namespace std;

void waitForEnterKey(){
    cin.ignore (numeric_limits<streamsize>::max (), '\n' );
}

int main (){

    cout << "Choose:"<<endl
         <<"        2: 2-axis calibration"<<endl
         <<"        3: 3-axis calibration"<<endl
         <<"        ::>"<<endl;

    int choice;
    cin >> choice;
    if(choice != 3){
        cout << "Please choose 3." << endl;
        return 0;
    }
    waitForEnterKey();

    cout << "Clearing any previous calibration data..." <<endl
         << "Press Enter to start sampling..." <<endl;

    waitForEnterKey();

    cout << "Sampling... Press Enter to stop..."<<endl;

    waitForEnterKey();

    cout << "Estimates: gains(0.000, 0.046, 0.007) offsets(26.752, 0.106, 0.412)"<<endl<<endl<<endl
         << "Compass calibration finished"<<endl
         << "    Vars: 56.846573, 26.752, 0.105974, 0.411768, 0.00230, 0.045985, 0.006789, 1.2000, 4.3242, 5.060, 7.23400, 1.20000, 0.9000"<<endl<<endl
         << "These arguments are in the right order to be passed direcly into the PhidgetSpatial_setCompassCorrectionParameters function."<<endl<<endl
         << "You may wish to use a more accurate value for magnetic field strength as the number provided here is only an estimate."<<endl<<endl
         << "Calibration values have been written to firmware on your spatial."<<endl
         << "These values will be maintained from now on, across power cycles, until explicitely reset or changed."<<endl<<endl
         << "Enter to exit";

    waitForEnterKey();
    return 0;
}

