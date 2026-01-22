# Bouncing-Balls
2D Multi-Ball Physics Simulation with Spin and Angled Ground.
Simulates elastic ball–ball collisions, wall interactions, and energy-dissipative bouncing on a finite angled surface, including rotational effects. MP4 animation output included for visualization and analysis.

Mathematical Model Description

This project implements a physics-based 2D multi-body dynamical system for particles under gravity with energy dissipation and spin-induced momentum transfer.

1️⃣ System Overview
Each particle is a rigid disk of radius r and mass m. The simulation incorporates:
• Uniform gravity: g = 9.81 m/s²
• Energy loss via coefficient of restitution (e)
• Ball-ball elastic collisions
• Spin-induced tangential momentum via angular velocity (ω)

2️⃣ Equations of Motion
Using Explicit Euler Integration:
• vᵢⁿ⁺¹ = vᵢⁿ + g Δt
• xᵢⁿ⁺¹ = xᵢⁿ + vᵢⁿ⁺¹ Δt

3️⃣ Geometry & Collisions
The angled ground is defined by inclination θ:
• y_g(x) = x tan(θ)
• Surface Normal: n̂ = (−sin θ, cos θ)

When a collision occurs (yᵢ ≤ y_g + r), velocity is decomposed into normal (vₙ) and tangential (vₜ) components:
• v' = vₜ − e vₙ

4️⃣ Rotational Effects
Spin (ω) introduces tangential impulse:
• v_x' ← v_x' + αωᵢ
Spin decays exponentially over time: ωᵢⁿ⁺¹ = λωᵢⁿ

5️⃣ Hard-Sphere Interaction
When distance ||xᵢ − xⱼ|| < 2r, an impulse-based correction enforces momentum conservation:
• vᵢ' = vᵢ + [−(1+e)vₙ / 2] n̂
