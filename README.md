![License: MIT](https://img.shields.io/badge/License-MIT-blue.svg)
![docs](https://img.shields.io/badge/docs-in_progress-yellow)
![bilingual](https://img.shields.io/badge/DE%2FEN-bilingual-orange)
![Contributions Welcome](https://img.shields.io/badge/contributions-welcome-brightgreen)

# AI‑Drones - A Bilingual Handbook on the Design, Construction, and Use of AI‑Enabled Drones

# AI‑Drohnen - Ein zweisprachiges Handbuch zu Entwurf, Bau und Einsatz von KI‑fähigen Drohnen

## Projektüberblick \| Project Overview

### 🇩🇪

Dieses Dokument bietet einen Einstieg in das komplexe Thema Drohnen mit künstlicher Intelligenz. Schwerpunkte sind die Entwicklung (einschließlich der Auswahl geeigneter Hard- und Softwarekomponenten), der Bau und der Betrieb von Drohnen in der Lehre sowie in Forschungsprojekten.

Die in diesem Dokument dargestellten Erkenntnisse stammen aus dem vom Connectom Vernetzungs- und Innovationsfonds des hessian.AI geförderten Forschungsprojekt *KI-gestützte Drohnenplattform* sowie aus der Lehrveranstaltung *Drohnen mit Künstlicher Intelligenz* an der Frankfurt University of Applied Sciences.

Maßgebliche Kriterien bei der Auswahl der in diesem Dokument vorgestellten Komponenten sind unter anderem die Anpassbarkeit an unterschiedliche Einsatzszenarien, die Anschaffungskosten, die Robustheit, die langfristige Marktverfügbarkeit sowie die Qualität der Dokumentation und des Hersteller-Supports.

Eine vollständige Abhandlung über Drohnen und KI ist nicht Ziel dieses Dokuments. Der Fokus liegt auf den Technologien und Lösungen, die zum Zeitpunkt der Erstellung aktuell waren und mit denen im Studienfeld Informatik des Fachbereichs 2 (Informatik und Ingenieurwissenschaften) der Frankfurt University of Applied Sciences praktische Erfahrungen gesammelt wurden.

Über Ihre Kommentare und Verbesserungsvorschläge freuen wir uns sehr.

### 🇬🇧

This document provides an introduction to the complex topic of drones with artificial intelligence. Its focus is on the development (including the selection of suitable hardware and software components), construction, and operation of drones for teaching and research purposes.

The insights presented in this document stem from the research project *AI-Assisted Drone Platform*, funded by the Connectom Networking and Innovation Fund of hessian.AI, as well as from the course *Drones with Artificial Intelligence* at the Frankfurt University of Applied Sciences.

Key criteria for selecting the components presented in this document include adaptability to different application scenarios, cost, robustness, long-term market availability, and the quality of documentation and manufacturer support.

The goal of this document is not to provide an exhaustive treatment of drones and AI, but rather to focus on technologies and solutions that were current at the time of its creation and for which practical experience was gained in the Computer Science program of Faculty 2 (Computer Science and Engineering) at the Frankfurt University of Applied Sciences.

We greatly appreciate your comments and suggestions for improvement.

## Struktur des Repositories \| Repository Structure

    ├── images/
    ├── chapter00.tex       # Vorwort 
    ├── chapter01.tex       # Hardware
    ├── chapter02.tex       # Software
    ├── chapter03.tex       # Wichtige Punkte
    ├── chapter04.tex       # Objekterkennung 
    ├── chapter05.tex       # Autopilot
    ├── chapter06.tex       # Follow-Me
    ├── chapter07.tex       # Drop-Mechanismen 
    ├── main.toc            # Englisches Inhaltsverzeichnis 
    ├── main.deutschestoc   # Deutsches Inhaltsverzeichnis 
    ├── main.tex            # Haupt-LaTeX-Datei (zweisprachig)
    ├── Makefile            # Skript zur Erstellung des Handbuchs
    ├── AI_Drones.pdf       # Kompilierte Dokumentausgabe (PDF) zum Teilen
    └── README.md           # Diese Datei / This file

## Nutzung \| Build Instructions

``` bash
git clone https://github.com/christianbaun/aidrones.git
make
```

## Lizenz \| License

Das komplette Werk ist unter der Creative Commons-Lizenz mit den Einschränkungen **Namensnennung** und **Weitergabe unter gleichen Bedingungen** in der Version 3.0 (CC-BY-SA-3.0) für Deutschland lizenziert.

[CC-BY-SA-3.0](https://creativecommons.org/licenses/by-sa/3.0/de/)

## Kontakt \| Contact

Über Ihre Kommentare und Verbesserungsvorschläge freuen wir uns sehr.

[Christian Baun](https://www.christianbaun.de/), 
[Matthias Deegener](https://www.frankfurt-university.de/de/erweiterungen/ansprechpartner/detail/matthias-deegener/), 
[Oliver Hahm](https://teaching.dahahm.de/), 
[Martin Kappes](https://fg-itsec.de/mitglieder/mitglieder-kappes/)
