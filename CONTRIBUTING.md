# Contributing to Isaac Sim Synthetic Data Pipeline

First off, thank you for considering contributing to this project! It's people like you that make the open source software community such an amazing place to learn, inspire, and create.

## 🤝 Code of Conduct

This project and everyone participating in it is governed by a Code of Conduct. By participating, you are expected to uphold this code.

## 🛠 Development Workflow

1.  **Fork the repository** on GitHub.
2.  **Clone your fork** locally: `git clone https://github.com/your-username/isaac-sim-synth-data.git`
3.  **Create a branch** for your feature: `git checkout -b feature/amazing-feature`
4.  **Make your changes**. Ensure you follow the C++17 coding standards.
5.  **Commit your changes** with meaningful commit messages.
6.  **Push to the branch**: `git push origin feature/amazing-feature`
7.  **Submit a Pull Request**.

## 📝 Coding Standards

- **C++ Version**: C++17
- **Naming Convention**:
    - Classes: `PascalCase` (e.g., `SceneBuilder`)
    - Functions: `PascalCase` (e.g., `LoadBaseScene`)
    - Variables: `camelCase` (e.g., `sensorData`)
    - Member variables: `camelCase` (e.g., `currentStage`)
- **Documentation**: All public classes and methods must be documented using Doxygen-style comments.

## 🧪 Testing

Please ensure your code compiles without warnings. In the future, unit tests (GTest) will be required for all logic commits.

## 🐞 Reporting Bugs

Bugs are tracked as GitHub issues. When filing an issue, please include:
- A clear title and description.
- Steps to reproduce the bug.
- The version of Isaac Sim you are using.
- Logs or screenshots if applicable.
