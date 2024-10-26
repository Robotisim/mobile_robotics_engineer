#include <iostream>

using namespace std;

void getTemperatureData(int *Templist, int size)
{
    // Pass const pointer to avoid accidental modification
    for (int i = 0; i < size; i++)
    {
        cout << "Temperature of the surronding according to sensor is :" << *(Templist + i) << "degree Farenheit" << endl;
    }
}

void getDistanceData(int *Dist)
{
    cout << "Distance from the ping sensor is:" << *Dist << "meter" << endl;
}

int main()
{
    int Temp[] = {30, 20, 44, 53, 61};
    int size = sizeof(Temp) / sizeof(Temp[0]);
    int Dist1 = 4;

    getTemperatureData(Temp, size); // Pass entire array
    getDistanceData(&Dist1);

    return 0;
}
