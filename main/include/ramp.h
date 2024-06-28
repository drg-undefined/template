#pragma once

class ramp {

    public:
        double get(double setPoint, double taxa);
        double calcula(double current = 0.0, double target = 0.0, double increment = 0.0);
    private:
        double tAnterior = 0.0;
        double out = 0.0;
        double aux_setPoint = 0.0;
        double taxa = 1.0;
        double tDecorrido = 0.0;

};