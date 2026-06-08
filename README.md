# ROV-stabilisation

Système de stabilisation automatique en profondeur d'un ROV sous-marin — TIPE 2025

---

## Description

Conception et réalisation d'un système de contrôle en profondeur pour un véhicule sous-marin téléopéré (ROV). Le système maintient automatiquement une profondeur de consigne grâce à un contrôleur PID implémenté sur Arduino MEGA.

---

## Matériel

| Composant | Référence |
|---|---|
| Microcontrôleur principal | Arduino MEGA |
| Microcontrôleur surface | Arduino NANO |
| Capteur de pression | BlueRobotics MS5837-30BA |
| Propulseur | BlueRobotics T100 |
| Variateur (ESC) | BlueRobotics Basic ESC |
| Communication | RS485 sur câble 20 m |

---

## Architecture du système

```
[Arduino NANO - Surface]
        |
    RS485 / 20 m
        |
[Arduino MEGA - ROV]
   ├── MS5837 (profondeur)  →  mesure
   └── T100 + ESC           →  actionneur
```

---

## Contrôleur PID

La commande est calculée à chaque cycle (50 ms) :

```
u(t) = Kp·e(t) + Ki·∫e(t)dt + Kd·(dz/dt)
```

- La dérivée est calculée sur la **mesure** (pas sur l'erreur) pour éviter les pics au changement de consigne
- L'intégrale est clampée entre −10 et +10 (anti-windup)
- Une rampe PWM de ±20 µs/cycle protège le propulseur

### Gains retenus après tests en piscine

| Paramètre | Valeur |
|---|---|
| Kp | 25.0 |
| Ki | 0.8 |
| Kd | 10.0 |

---

## Fonctionnalités

- Maintien de profondeur avec erreur statique nulle
- Sécurité timeout liaison (5 s sans consigne → moteur neutre)
- Filtre des valeurs aberrantes du capteur
- Calibration automatique de l'offset à la mise en route
- Séquence d'armement ESC sécurisée (attente confirmation manuelle)

---

## Utilisation

1. Alimenter l'Arduino MEGA
2. Ouvrir le moniteur série (9600 bauds)
3. Brancher la batterie de l'ESC
4. Attendre `ROV pret`
5. Régler la consigne de profondeur via le potentiomètre sur l'Arduino NANO

---

## Simulation

Le système a été modélisé et simulé sous **Scilab/Xcos** avant les tests réels.  
Les données expérimentales ont été traitées avec **Python**.

---

## Auteur

Ivan Le Roux — Classe préparatoire PTSI/PT, La Croix-Rouge, Brest
