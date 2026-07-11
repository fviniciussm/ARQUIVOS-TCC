#ifndef INPUT_H
#define INPUT_H

// ===================== Entradas do usuário ====================== //

// Estrutura que guarda os dois valores
struct ValoresVelocidade {
    float setvel;    // valor em ajuste
    float setpoint;  // valor confirmado
};

void initInput();
ValoresVelocidade EntradasVelocidade();

void ResetEntradasVelocidade();

#endif
