---
jupytext:
  formats: md:myst
  text_representation:
    extension: .md
    format_name: myst
kernelspec:
  display_name: Python 3
  language: python
  name: python3
---

# Quizzes

```{code-cell} ipython3
:tags: ["hide-input"]

from jupyterquiz import display_quiz
import json, requests
git_path="https://raw.githubusercontent.com/j-albr16/asumr/main/quizzes/localisation.json"
r = requests.get(git_path)
display_quiz(r.json())
```


