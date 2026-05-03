## Equations

Puisque vous supposez avoir les vitesses de rotation des moteurs ($\Omega_1, \Omega_2, \Omega_3, \Omega_4$), vous devez calculer le vecteur de commande $U$ :

*   **Poussée totale ($U_1$) :** Somme des poussées individuelles.
    *   $U_1 = b \times (\Omega_1^2 + \Omega_2^2 + \Omega_3^2 + \Omega_4^2)$
*   **Moment de Roll ($U_2$) :** Différence entre les moteurs latéraux.
    *   $U_2 = b \times (-\Omega_2^2 + \Omega_4^2)$
*   **Moment de Pitch ($U_3$) :** Différence entre moteurs avant et arrière.
    *   $U_3 = b \times (\Omega_1^2 - \Omega_3^2)$
*   **Moment de Yaw ($U_4$) :** Différence de couple entre paires horaires et anti-horaires.
    *   $U_4 = d \times (-\Omega_1^2 + \Omega_2^2 - \Omega_3^2 + \Omega_4^2)$

