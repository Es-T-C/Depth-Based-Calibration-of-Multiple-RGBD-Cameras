# Depth-Based-Calibration-of-Multiple-RGBD-Cameras
Cite **Novel Methods for Depth-Based Calibration of Multiple RGBD Cameras using Four Mutually Equidistant Spheres**
doi: 10.1109/JSEN.2025.3616445 

- This project performs **depth-only calibration** of four RGB-D cameras using a custom-made calibration object consisting of **four equal-radius spheres**.  
- Two novel optimization methods inspired by **Bundle Adjustment** are proposed, operating directly on **3D coordinates** computed from depth data.

---

## 📌 Project Summary

- 🎯 Calibrates four RGB-D depth cameras using only depth images.
- 🟠 Uses a tetrahedron calibration object with **4 spheres**.
- 🧠 Proposes two new **depth-based 3D optimization algorithms** that outperform traditional techniques.
- 📉 Achieves **significantly lower 3D reconstruction errors** compared to conventional methods.

---

## 📁 Dataset

The dataset includes flatWall and object depth images, and some necessary files. Due to its size, it is hosted externally:

🔗 [Download Dataset (Google Drive)](https://drive.google.com/drive/folders/1CCt7ZfGrxcs1ux4DplRUCF0r-M7Twyjh?usp=sharing)

---

## 💻 Requirements

This project is implemented entirely in **MATLAB**. The following toolboxes are required:

- ✅ **Image Processing Toolbox**  
  
- ✅ **Optimization Toolbox**  
  
- ✅ **Computer Vision Toolbox**  
  
> MATLAB R2024a or later is recommended for compatibility.

---

## ⚙️ Installation

Clone the repository and open it in MATLAB:

```bash
git clone https://github.com/Es-T-C/Depth-Based-Calibration-of-Multiple-RGBD-Cameras.git
cd Depth-Based-Calibration-of-Multiple-RGBD-Cameras

