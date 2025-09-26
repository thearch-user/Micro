
---

# 🛠️ Git Commands to Publish

Run these in your terminal (replace `<your-username>` with your GitHub username):

```bash
# create new folder and move code into it
mkdir Micro
cd Micro

# initialize git repo
git init

# add your code file and README
cp /path/to/fullstack_enterprise_embedded.py .
echo "# Micro" > README.md   # (optional if you already use the one I wrote)

# stage and commit
git add .
git commit -m "Initial commit: ESP32 + Pico unified firmware"

# create new repo on GitHub (through browser: github.com -> New Repo -> name: Micro)

# add remote (replace with your username)
git remote add origin https://github.com/<your-username>/Micro.git

# push code to GitHub
git branch -M main
git push -u origin main
