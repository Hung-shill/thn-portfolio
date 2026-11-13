# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

This is a personal portfolio website for Hung Nguyen, a Mechanical Engineering student at Loyola Marymount University. The site is built as a static HTML/CSS/JavaScript website and deployed to GitHub Pages at https://hung-shill.github.io/thn-portfolio/.

## Architecture

- **Static Website**: Pure HTML, CSS, and JavaScript - no build process or frameworks
- **Single Page Application**: Main content in `index.html` with additional project pages
- **GitHub Pages Deployment**: Automatically deployed via GitHub Actions on push to main branch
- **Responsive Design**: Mobile-first approach with CSS custom properties for theming

## Key Files and Structure

- `index.html` - Main portfolio page with sections: hero, projects, experience, about and skills, contact
- `project.html` - Redirects to `index.html#projects` (projects now live on the home page)
- `css/home.css` - Main stylesheet: color, type, and layout tokens on `:root`, plus header, footer, and home sections
- `css/project-page.css` - Styles for project detail pages, loaded after `home.css`
- `CoursesDesignPortfolio.html` - Redirects to `index.html#projects` (course photos moved to FSAE, research, and Autonomous Pookie pages)
- `images/` - All portfolio images and icons
- Additional project pages: `FSAE.html`, `parkingpal.html`, `autonomousRC.html`, etc.

## Development Commands

**Local Development:**

- Use VS Code Live Server extension or any local HTTP server
- Configured for Chrome debugging on `localhost:8080` (see `.vscode/launch.json`)

**Code Formatting:**

- Prettier configuration: 2-space tabs, no semicolons, double quotes
- Format with: `npx prettier --write .` (if prettier is installed)

**Deployment:**

- Automatic deployment via GitHub Actions on push to `main` branch
- No manual build step required - static files are deployed directly

## Development Notes

- **No Package Manager**: This is a vanilla HTML/CSS/JS project with no package.json or build tools
- **Raspberry Pi Target**: Project is developed on macOS but intended for deployment to Raspberry Pi 5 for a "mini Waymo project"
- **Portfolio Focus**: Content emphasizes mechanical engineering, CAD skills, autonomous vehicles, and hands-on projects
- **Image Assets**: Extensive use of skill icons, project images, and personal photos in `images/` directory

## Making Changes

When editing:

- Follow the existing CSS custom property system for consistent theming
- Maintain responsive design patterns used throughout
- Keep the clean, professional aesthetic established in the current design
- Test changes locally before pushing (automatic deployment on main branch)
- these files are my personal mechanical engineering portfolio
- always ask for clarification
- always use descriptive variable names
- keep the <link rel="icon" type="image/png" href="./images/favicon.png" /> (HN monogram) on every page
- always point to code snippet when explaining something