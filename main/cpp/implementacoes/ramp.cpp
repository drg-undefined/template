
#include "ramp.h"
#include <frc/Timer.h>

double ramp::get(double setPoint, double taxa = 1.0){
    
    double tAtual = frc::Timer::GetFPGATimestamp();

    if (setPoint != aux_setPoint){
    // //     taxa = std::abs(aux_setPoint - setPoint) / dT;
        aux_setPoint = setPoint;
    //     tDecorrido = 0.0;
        tAnterior = tAtual;
    }
    
    if (setPoint > out) {
        double deltaT = tAtual - tAnterior;
        // Aumentar a velocidade atual gradualmente
        // out = std::min(out + taxa * deltaT, setPoint);
        out = std::min(out + taxa, setPoint);
    } else if (setPoint < out) {
        double deltaT = tAtual - tAnterior;
         // Reduzir a velocidade atual gradualmente
        // out = std::max(out - taxa * deltaT, setPoint);
        out = std::max(out - taxa, setPoint);
    }

    // tDecorrido += deltaT;
    // if (tDecorrido <= dT) {
    //     out = taxa * tDecorrido;
    // } else {
    //     out = setPoint;
    // }

    return out;
}

double ramp::calcula(double current, double target, double increment) {
    if (current < target) {
        current += increment;
        if (current > target) {
            current = target;
        }
    } else if (current > target) {
        current -= increment;
        if (current < target) {
            current = target;
        }
    }
    return current;
}