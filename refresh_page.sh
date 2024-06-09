#!/bin/bash

# check for arguments
while getopts d: flag
do
    case "${flag}" in
        d) decker=${OPTARG};;
    esac
done

# change to the root directory of the repository
git checkout main
git pull

# check if the virtual environment exists
if [ ! -d "venv" ]; then
    echo "Virtual environment not found. Setting up .."
    python3 -m venv venv
    echo "Virtual environment set up. Activating .."
    source venv/bin/activate
    echo "Installing requirements .."
    python3 -m pip install -r requirements.txt
fi

# activate the virtual environment
source venv/bin/activate

# merge the main branch into the gh-pages branch
git checkout gh-pages
git pull
git merge main --no-edit

# rebuild the page
make rebuild

# check if the decker page should be refreshed
if [ "$decker" = "force" ]; then
    echo "Refreshing the decker page .."
    rm -rf decker_slides/
    cp -r ../asmr_private/decks4ex/public/ decker_slides/
elif [ "$decker" = "none" ]; then
    echo "Skipping the decker page refresh .."
else
    # ask if decker should be refreshed
    read -p "Do you want to refresh the decker page? (y/n) " -n 1 -r
    echo
    if [[ $REPLY =~ ^[Yy]$ ]]; then
        echo "Refreshing the decker page .."
        rm -rf decker_slides/
        cp -r ../asmr_private/decks4ex/public/ decker_slides/
    fi
fi

# copy the slides into the jupyterbook folder
cp -r decker_slides/ docs/ex_slides/

# pushcommit the changes
git add .
git commit -m "refresh page"
git push

# switch back to the main branch
git checkout main
