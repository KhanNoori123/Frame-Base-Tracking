# Contributing to Drone Tracking System

Thank you for your interest in contributing! This document provides guidelines for contributing to the project.

## Development Setup

1. Fork the repository
2. Clone your fork:
```bash
git clone https://github.com/yourusername/drone-tracking-system.git
cd drone-tracking-system
```

3. Create a virtual environment:
```bash
python -m venv venv
source venv/bin/activate  # On Windows: venv\Scripts\activate
```

4. Install dependencies:
```bash
pip install -r requirements.txt
```

## Project Structure

- `src/core/`: Core system components (camera management)
- `src/tracking/`: Object detection and tracking algorithms
- `src/control/`: Drone control and IBVS algorithms
- `src/ui/`: User interface and visualization
- `src/simulation/`: Gazebo simulation integration
- `config/`: Configuration files
- `tests/`: Test scripts
- `docs/`: Documentation

## Coding Standards

- Follow PEP 8 style guide
- Use meaningful variable and function names
- Add docstrings to all functions and classes
- Keep functions focused and modular
- Comment complex logic

## Making Changes

1. Create a new branch:
```bash
git checkout -b feature/your-feature-name
```

2. Make your changes
3. Test your changes thoroughly
4. Commit with clear messages:
```bash
git commit -m "Add: brief description of changes"
```

5. Push to your fork:
```bash
git push origin feature/your-feature-name
```

6. Create a Pull Request

## Testing

Before submitting a PR:
- Test with both webcam and UDP stream
- Verify tracking performance
- Check drone control functionality
- Test in simulation if possible

## Documentation

- Update relevant documentation in `docs/`
- Add comments for complex algorithms
- Update README.md if adding new features

## Questions?

Open an issue for questions or discussions.
