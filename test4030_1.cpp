// 畢專.cpp : 此檔案包含 'main' 函式。程式會於該處開始執行及結束執行。

#include <iostream>
#include <fstream>
#include <string>
#include "gurobi_c++.h"
using namespace std;

int main()
{
    // 設參數(暫時)
    ////////////
    const int v_num = 14; // stands for vessels' number(k)
    const int Port = 24;  // stands for port's length(i)
    const int Time = 48;  // stands for time(j)
    double c1[v_num + 1]{0};
    double c2[v_num + 1]{0};
    double c3[v_num + 1]{0};
    double l[v_num + 1]{0}; // length of vessel k(Bk)
    double p[v_num + 1]{0}; // prefer position of vessel k(Sk)
    double e[v_num + 1]{0};
    double d[v_num + 1]{0};
    double r[v_num + 1]{0}; // require time
    const int M = 999;

    // 讀檔
    fstream fin;
    cout << "e\n";
    fin.open("D:/交大/運管畢專/Gurobi/程式/資料集/v1/ek.txt", ios::in);
    if (!fin)
    {
        cout << "Fail\n";
        system("pause");
        exit(1);
    }
    for (int k = 1; k <= v_num; k++)
    {
        fin >> e[k];
        cout << e[k] << "\t";
    }
    cout << "\n";
    fin.close();

    cout << "l\n";
    fin.open("D:/交大/運管畢專/Gurobi/程式/資料集/v1/lk.txt", ios::in);
    if (!fin)
    {
        cout << "Fail\n";
        system("pause");
        exit(1);
    }
    for (int k = 1; k <= v_num; k++)
    {
        fin >> l[k];
        cout << l[k] << "\t";
    }
    cout << "\n";
    fin.close();

    cout << "d\n";
    fin.open("D:/交大/運管畢專/Gurobi/程式/資料集/v1/dk.txt", ios::in);
    if (!fin)
    {
        cout << "Fail\n";
        system("pause");
        exit(1);
    }
    for (int k = 1; k <= v_num; k++)
    {
        fin >> d[k];
        cout << d[k] << "\t";
    }
    cout << "\n";
    fin.close();

    cout << "p\n";
    fin.open("D:/交大/運管畢專/Gurobi/程式/資料集/v1/pk.txt", ios::in);
    if (!fin)
    {
        cout << "Fail\n";
        system("pause");
        exit(1);
    }
    for (int k = 1; k <= v_num; k++)
    {
        fin >> p[k];
        cout << p[k] << "\t";
    }
    cout << "\n";
    fin.close();

    cout << "r\n";
    fin.open("D:/交大/運管畢專/Gurobi/程式/資料集/v1/rk.txt", ios::in);
    if (!fin)
    {
        cout << "Fail\n";
        system("pause");
        exit(1);
    }
    for (int k = 1; k <= v_num; k++)
    {
        fin >> r[k];
        cout << r[k] << "\t";
    }
    cout << "\n";
    fin.close();

    cout << "c1\n";
    fin.open("D:/交大/運管畢專/Gurobi/程式/資料集/v1/c1.txt", ios::in);
    if (!fin)
    {
        cout << "Fail\n";
        system("pause");
        exit(1);
    }
    for (int k = 1; k <= v_num; k++)
    {
        fin >> c1[k];
        cout << c1[k] << "\t";
    }
    cout << "\n";
    fin.close();

    cout << "c2\n";
    fin.open("D:/交大/運管畢專/Gurobi/程式/資料集/v1/c2.txt", ios::in);
    if (!fin)
    {
        cout << "Fail\n";
        system("pause");
        exit(1);
    }
    for (int k = 1; k <= v_num; k++)
    {
        fin >> c2[k];
        cout << c2[k] << "\t";
    }
    cout << "\n";
    fin.close();

    cout << "c3\n";
    fin.open("D:/交大/運管畢專/Gurobi/程式/資料集/v1/c3.txt", ios::in);
    if (!fin)
    {
        cout << "Fail\n";
        system("pause");
        exit(1);
    }
    for (int k = 1; k <= v_num; k++)
    {
        fin >> c3[k];
        cout << c3[k] << "\t";
    }
    cout << "\n";
    fin.close();

    // 程式主體
    try
    {
        string name;
        GRBEnv env = GRBEnv();
        GRBModel model = GRBModel(env);

        // 設變數
        // v1(xijk):being 1 if vessel k cover the space i in time j
        GRBVar x[Port + 1][Time + 1][v_num + 1];
        for (int i = 1; i <= Port; i++)
        {
            for (int j = 1; j <= Time; j++)
            {
                for (int k = 1; k <= v_num; k++)
                {
                    name = "x_i" + to_string(i) + "_j" + to_string(j) + "_k" + to_string(k);
                    x[i][j][k] = model.addVar(0, 1, 0.0, GRB_BINARY, name);
                }
            }
        }

        // v2(zijk):being 1 if the berthing is perfect
        GRBVar z[Port + 1][Time + 1][v_num + 1];
        for (int i = 1; i <= Port; i++)
        {
            for (int j = 1; j <= Time; j++)
            {
                for (int k = 1; k <= v_num; k++)
                {
                    name = "z_i" + to_string(i) + "_j" + to_string(j) + "_k" + to_string(k);
                    z[i][j][k] = model.addVar(0, 1, 0.0, GRB_BINARY, name);
                }
            }
        }

        // v3(vjk):being 1 if vessel k is operating in time j
        GRBVar v[Time + 1][v_num + 1];
        for (int j = 1; j <= Time; j++)
        {
            for (int k = 1; k <= v_num; k++)
            {
                name = "v_j" + to_string(j) + "_k" + to_string(k);
                v[j][k] = model.addVar(0, 1, 0.0, GRB_BINARY, name);
            }
        }

        // v4(uik):being 1 if vessel k berth in position i
        GRBVar u[Port + 1][v_num + 1];
        for (int i = 1; i <= Port; i++)
        {
            for (int k = 1; k <= v_num; k++)
            {
                name = "u_i" + to_string(i) + "_k" + to_string(k);
                u[i][k] = model.addVar(0, 1, 0.0, GRB_BINARY, name);
            }
        }

        // v5(ck):completion time of container handling for vessel k
        GRBVar c[v_num + 1];
        for (int k = 1; k <= v_num; k++)
        {
            name = "c_k" + to_string(k);
            c[k] = model.addVar(0, INFINITY, 0.0, GRB_CONTINUOUS, name);
        }

        // v6(b_lk):most left position for vessel k to berth
        GRBVar b_l[v_num + 1];
        for (int k = 1; k <= v_num; k++)
        {
            name = "bl_k" + to_string(k);
            b_l[k] = model.addVar(0, INFINITY, 0.0, GRB_CONTINUOUS, name);
        }

        // v7(b_rk):most right position for vessel k to berth
        GRBVar b_r[v_num + 1];
        for (int k = 1; k <= v_num; k++)
        {
            name = "br_k" + to_string(k);
            b_r[k] = model.addVar(0, INFINITY, 0.0, GRB_CONTINUOUS, name);
        }

        // v8(d_ck):delay amount of berthing time for vessel k(TLk)
        GRBVar d_c[v_num + 1];
        for (int k = 1; k <= v_num; k++)
        {
            name = "dc_k" + to_string(k);
            d_c[k] = model.addVar(0, INFINITY, 0.0, GRB_CONTINUOUS, name);
        }

        // v9(d_lk):delay amount of leaving time for vessel k
        GRBVar d_l[v_num + 1];
        for (int k = 1; k <= v_num; k++)
        {
            name = "dl_k" + to_string(k);
            d_l[k] = model.addVar(0, INFINITY, 0.0, GRB_CONTINUOUS, name);
        }

        // v10(berthk):(Bk)
        GRBVar berth[v_num + 1];
        for (int k = 1; k <= v_num; k++)
        {
            name = "berth_k" + to_string(k);
            berth[k] = model.addVar(0, INFINITY, 0.0, GRB_CONTINUOUS, name);
        }

        // v11(timek):(Tk)
        GRBVar time[v_num + 1];
        for (int k = 1; k <= v_num; k++)
        {
            name = "time_k" + to_string(k);
            time[k] = model.addVar(0, INFINITY, 0.0, GRB_CONTINUOUS, name);
        }

        // 目標式
        GRBLinExpr sum = 0;
        GRBLinExpr sum1 = 0;
        GRBLinExpr sum2 = 0;
        GRBLinExpr sum3 = 0;
        GRBLinExpr sumA = 0;
        GRBLinExpr sumB = 0;

        for (int k = 1; k <= v_num; k++)
        {
            sum1 += ((b_l[k] + b_r[k]) * c1[k]);
        }
        for (int k = 1; k <= v_num; k++)
        {
            sum2 += (d_l[k] * c2[k]);
        }
        for (int k = 1; k <= v_num; k++)
        {
            sum3 += (d_c[k] * c3[k]);
        }

        sum = sum1 + sum2 + sum3;
        model.setObjective(sum, GRB_MINIMIZE);
        cout << "object\n";

        // 限制式

        // s.t.5 位置左偏時給予懲罰值
        cout << "s.t.5\t";
        for (int k = 1; k <= v_num; k++)
        {
            name = "st5_" + to_string(k);
            model.addConstr(b_l[k] >= p[k] - berth[k], name);
        }
        cout << "success~\n";

        // s.t.6 位置右偏時給予懲罰值
        cout << "s.t.6\t";
        for (int k = 1; k <= v_num; k++)
        {
            name = "st6_" + to_string(k);
            model.addConstr(b_r[k] >= berth[k] - p[k], name);
        }
        cout << "success~\n";

        // s.t.7 抵達時間penalty
        cout << "s.t.7\t";
        for (int k = 1; k <= v_num; k++)
        {
            name = "st7_" + to_string(k);
            model.addConstr(d_c[k] >= time[k] - e[k], name);
        }
        cout << "success~\n";

        // s.t.8 限制船隻不能早到
        cout << "s.t.8\t";
        for (int k = 1; k <= v_num; k++)
        {
            name = "st8_" + to_string(k);
            model.addConstr(time[k] >= e[k], name);
        }
        cout << "success~\n";

        // s.t.9 離開時間延誤penalty
        cout << "s.t.9\t";
        for (int k = 1; k <= v_num; k++)
        {
            name = "st9_" + to_string(k);
            model.addConstr(d_l[k] >= c[k] - d[k], name);
        }
        cout << "success~\n";

        // s.t.10 ****
        cout << "s.t.10\t";
        for (int k = 1; k <= v_num; k++)
        {
            for (int j = 1; j <= Time; j++)
            {
                name = "st10_" + to_string(k);
                model.addConstr(c[k] >= (v[j][k] * (j + 1)), name);
            }
        }
        cout << "success~\n";

        // s.t.11
        cout << "s.t.11\t";
        for (int i = 1; i <= Port; i++)
        {
            for (int j = 1; j <= Time; j++)
            {
                sumA = 0;
                for (int k = 1; k <= v_num; k++)
                {
                    sumA += x[i][j][k];
                }
                name = "st11_" + to_string(i) + "_" + to_string(j);
                model.addConstr(sumA <= 1, name); //(對每一對ij，x都只會分配給一個k或是不分配)
            }
        }
        cout << "success~\n";

        // s.t.18 ////應該要加，但論文中把V寫成Y
        cout << "s.t.18\t";
        for (int j = 1; j <= Time; j++)
        {
            for (int k = 1; k <= v_num; k++)
            {
                sumA = 0;
                for (int i = 1; i <= Port; i++)
                {
                    sumA += x[i][j][k];
                }
                name = "st18_" + to_string(j) + "_" + to_string(k);
                model.addConstr(v[j][k] <= sumA, name);
            }
        }
        cout << "success~\n";

        // s.t.19
        cout << "s.t.19\t";
        for (int j = 1; j <= Time; j++)
        {
            for (int k = 1; k <= v_num; k++)
            {
                sumA = 0;
                for (int i = 1; i <= Port; i++)
                {
                    sumA += x[i][j][k];
                }
                name = "st19_" + to_string(j) + "_" + to_string(k);
                model.addConstr(M * v[j][k] >= sumA, name);
            }
        }
        cout << "success~\n";

        // s.t.20
        cout << "s.t.20\t";
        for (int i = 1; i <= Port; i++)
        {
            for (int k = 1; k <= v_num; k++)
            {
                sumA = 0;
                for (int j = 1; j <= Time; j++)
                {
                    sumA += x[i][j][k];
                }
                name = "st20_" + to_string(i) + "_" + to_string(k);
                model.addConstr(u[i][k] <= sumA, name);
            }
        }
        cout << "success~\n";

        // s.t.21
        cout << "s.t.21\t";
        for (int i = 1; i <= Port; i++)
        {
            for (int k = 1; k <= v_num; k++)
            {
                sumA = 0;
                for (int j = 1; j <= Time; j++)
                {
                    sumA += x[i][j][k];
                }
                name = "st21_" + to_string(i) + "_" + to_string(k);
                model.addConstr(M * u[i][k] >= sumA, name);
            }
        }
        cout << "success~\n";

        // s.t.22
        cout << "s.t.22\t";
        for (int k = 1; k <= v_num; k++)
        {
            for (int i_a = 1; i_a <= Port; i_a++)
            {
                for (int i_b = 1; i_b <= Port; i_b++)
                {
                    if (i_a > 1 and i_b > i_a and Port > i_b)
                    {
                        sumA = 0;
                        for (int i = i_a; i <= i_b; i++)
                        {
                            sumA += u[i][k];
                        }
                        name = "st22_" + to_string(i_a) + "_" + to_string(i_b) + "_" + to_string(k);
                        model.addConstr(i_b - i_a + 1 <= sumA + M * (2 - u[i_a][k] - u[i_b][k]), name);
                    }
                }
            }
        }
        cout << "success~\n";

        // s.t.23
        cout << "s.t.23\t";
        for (int k = 1; k <= v_num; k++)
        {
            for (int j_a = 1; j_a <= Time; j_a++)
            {
                for (int j_b = 1; j_b <= Time; j_b++)
                {
                    if (j_a > 1 and j_b > j_a and Time > j_b)
                    {
                        sumA = 0;
                        for (int j = j_a; j <= j_b; j++)
                        {
                            sumA += v[j][k];
                        }
                        name = "st23_" + to_string(j_a) + "_" + to_string(j_b) + "_" + to_string(k);
                        model.addConstr(j_b - j_a + 1 <= sumA + M * (2 - v[j_a][k] - v[j_b][k]), name);
                    }
                }
            }
        }
        cout << "success~\n";

        // s.t.24
        cout << "s.t.24\t";
        for (int j = 1; j <= Time; j++)
        {
            for (int k = 1; k <= v_num; k++)
            {
                sumA = 0;
                for (int i = 1; i <= Port; i++)
                {
                    for (int j_a = 1; j_a <= j; j_a++)
                    {
                        sumA += z[i][j_a][k];
                    }
                }
                name = "st24_" + to_string(j) + "_" + to_string(k);
                model.addConstr(v[j][k] <= sumA, name);
            }
        }
        cout << "success~\n";

        // s.t.25
        cout << "s.t.25\t";
        for (int k = 1; k <= v_num; k++)
        {
            sumA = 0;
            for (int i = 1; i <= Port; i++)
            {
                for (int j = 1; j <= Time; j++)
                {
                    sumA += z[i][j][k];
                }
            }
            name = "st25_" + to_string(k);
            model.addConstr(sumA == 1, name);
        }
        cout << "success~\n";

        // s.t.26
        cout << "s.t.26\t";
        for (int k = 1; k <= v_num; k++)
        {
            for (int i_a = 2; i_a <= Port; i_a++)
            {
                if (i_a <= (Port - l[k]))
                {
                    sumA = 0;
                    sumB = 0;
                    for (int i = 1; i <= Port; i++)
                    {
                        for (int j = 1; j <= Time; j++)
                        {
                            if (i < i_a or i >= (i_a + l[k]))
                            {
                                sumA += x[i][j][k];
                            }
                        }
                    }
                    for (int j = 1; j <= Time; j++)
                    {
                        sumB += z[i_a][j][k];
                    }
                    name = "st26_" + to_string(k) + "_" + to_string(i_a);
                    model.addConstr(sumA <= M * (1 - sumB), name);
                }
            }
        }
        cout << "success~\n";

        // s.t.27
        cout << "s.t.27\t";
        for (int k = 1; k <= v_num; k++)
        {
            sumA = 0;
            sumB = 0;
            for (int i = 1; i <= Port; i++)
            {
                for (int j = 1; j <= Time; j++)
                {
                    if (i > l[k])
                    {
                        sumA += x[i][j][k];
                    }
                }
            }
            for (int j = 1; j <= Time; j++)
            {
                sumB += z[1][j][k];
            }
            name = "27_" + to_string(k);
            model.addConstr(sumA <= M * (1 - sumB), name);
        }
        cout << "success~\n";

        // s.t.28
        cout << "s.t.28\t";
        for (int k = 1; k <= v_num; k++)
        {
            sumA = 0;
            sumB = 0;
            for (int i = 1; i <= Port; i++)
            {
                for (int j = 1; j <= Time; j++)
                {
                    if (i >= Port - l[k])
                    {
                        sumA += x[i][j][k];
                    }
                }
            }
            int test = Port - l[k] + 1;
            for (int j = 1; j <= Time; j++)
            {
                sumB += z[test][j][k];
            }
            name = "28_" + to_string(k);
            model.addConstr(sumA <= M * (1 - sumB), name);
        }
        cout << "success~\n";

        // s.t.29
        cout << "s.t.29\t";
        for (int k = 1; k <= v_num; k++)
        {
            for (int j = 1; j <= Time; j++)
            {
                sumA = 0;
                for (int i = 1; i <= Port; i++)
                {
                    sumA += x[i][j][k];
                }
                name = "st29_" + to_string(k);
                model.addConstr(l[k] - sumA <= M * (1 - v[j][k]), name);
            }
        }
        cout << "success~\n";

        // s.t.sp1(Bk)
        cout << "s.t.sp1\t";
        for (int k = 1; k <= v_num; k++)
        {
            sumA = 0;
            for (int i = 1; i <= Port; i++)
            {
                for (int j = 1; j <= Time; j++)
                {
                    sumA += (z[i][j][k] * i);
                }
            }
            name = "sp1_" + to_string(k);
            model.addConstr(berth[k] == sumA, name);
        }
        cout << "success~\n";

        // s.t.sp2(Tk)
        cout << "s.t.sp2\t";
        for (int k = 1; k <= v_num; k++)
        {
            sumA = 0;
            for (int i = 1; i <= Port; i++)
            {
                for (int j = 1; j <= Time; j++)
                {
                    sumA += (z[i][j][k] * j);
                }
            }
            name = "sp2_" + to_string(k);
            model.addConstr(time[k] == sumA, name);
        }
        cout << "success~\n";

        // s.t.sp3(ck = Tk + rk) /////////有改
        cout << "s.t.sp3\t";
        for (int k = 1; k <= v_num; k++)
        {
            name = "sp3_" + to_string(k);
            model.addConstr(c[k] == time[k] + r[k], name);
        }
        cout << "success~\n";

        // s.t.sp4(Z)
        cout << "s.t.sp4\t";
        for (int k = 1; k <= v_num; k++)
        {
            sumA = 0;
            for (int i = 1; i <= Port; i++)
            {
                for (int j = 1; j <= Time; j++)
                {
                    sumA += x[i][j][k];
                }
            }
            name = "sp4_" + to_string(k);
            model.addConstr(sumA == l[k] * r[k], name);
        }
        cout << "success~\n";

        // s.t.sp6(船不重疊)
        cout << "s.t.sp6\t";
        for (int i = 1; i <= Port; i++)
        {
            for (int j = 1; j <= Time; j++)
            {
                sumA = 0;
                for (int k = 1; k <= v_num; k++)
                {
                    sumA += x[i][j][k];
                }
                name = "sp6_" + to_string(i) + "_" + to_string(j);
                model.addConstr(sumA <= 1, name);
            }
        }
        cout << "success~\n";

        model.optimize();

        for (int k = 1; k <= v_num; k++)
        {

            cout << berth[k].get(GRB_DoubleAttr_X) << ",";
        }
        cout << "\n";
        for (int k = 1; k <= v_num; k++)
        {

            cout << time[k].get(GRB_DoubleAttr_X) << ",";
        }
        cout << "\n";
        for (int k = 1; k <= v_num; k++)
        {

            cout << c[k].get(GRB_DoubleAttr_X) << ",";
        }
        cout << "\n";
        for (int k = 1; k <= v_num; k++)
        {

            cout << d_c[k].get(GRB_DoubleAttr_X) << ",";
        }
        cout << "\n";
        for (int k = 1; k <= v_num; k++)
        {

            cout << d_l[k].get(GRB_DoubleAttr_X) << ",";
        }
        cout << "\n";
        /*for (int k = 1; k <= v_num; k++) {
            for (int i = 1; i <= Port; i++) {
                for (int j = 1; j <= Time; j++) {
                    cout << z[i][j][k].get(GRB_DoubleAttr_X) << ",";
                }
                cout << "\n";
            }
            cout << "\n";
        }
        cout << "\n";
        for (int k = 1; k <= v_num; k++) {
            for (int i = 1; i <= Port; i++) {
                for (int j = 1; j <= Time; j++) {
                    cout << x[i][j][k].get(GRB_DoubleAttr_X) << ",";
                }
                cout << "\n";
            }
            cout << "\n";
        }*/
        for (int i = 1; i <= Port; i++)
        {
            for (int j = 1; j <= Time; j++)
            {
                int check = 0;
                for (int k = 1; k <= v_num; k++)
                {
                    check += x[i][j][k].get(GRB_DoubleAttr_X);
                }
                switch (check)
                {
                case 1:
                    cout << "■";
                    break;
                case 0:
                    cout << "□";
                    break;
                default:
                    cout << "▣";
                    break;
                }
                // cout << check << ",";
            }
            cout << "\n";
        }
    }
    catch (GRBException err)
    {
        cout << "Error code = " << err.getErrorCode() << endl;
        cout << err.getMessage() << endl;
    }
    catch (...)
    {
        cout << "Exception during optimization" << endl;
    }

    system("pause");
    return 0;
}

// 執行程式: Ctrl + F5 或 [偵錯] > [啟動但不偵錯] 功能表
// 偵錯程式: F5 或 [偵錯] > [啟動偵錯] 功能表

// 開始使用的提示:
//   1. 使用 [方案總管] 視窗，新增/管理檔案
//   2. 使用 [Team Explorer] 視窗，連線到原始檔控制
//   3. 使用 [輸出] 視窗，參閱組建輸出與其他訊息
//   4. 使用 [錯誤清單] 視窗，檢視錯誤
//   5. 前往 [專案] > [新增項目]，建立新的程式碼檔案，或是前往 [專案] > [新增現有項目]，將現有程式碼檔案新增至專案
//   6. 之後要再次開啟此專案時，請前往 [檔案] > [開啟] > [專案]，然後選取 .sln 檔案
