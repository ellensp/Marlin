#!/usr/bin/env python
#
# fetch_config_gui.py
#

import os, subprocess, sys, urllib.request, argparse, requests
import tkinter as tk

DEBUG = False

def fetch_configs(branch, config_path):
  print(f"Fetching {config_path} configurations from {branch}...")
  config_path = config_path.replace(' ', '%20')
  base_url = f"https://raw.githubusercontent.com/MarlinFirmware/Configurations/{branch}/config/{config_path}"
  files = ["Configuration.h", "Configuration_adv.h", "_Bootscreen.h", "_Statusscreen.h"]
  marlin_dir = os.path.join(os.getcwd(), "Marlin")
  if not os.path.exists(marlin_dir):
    print(f"Directory {marlin_dir} does not exist.")
    sys.exit(1)
  for file in files:
    url = f"{base_url}/{file}"
    dest_file = os.path.join(marlin_dir, file)
    if DEBUG:
      print(f"Fetching {file} from {url} to {dest_file}")
    try:
      urllib.request.urlretrieve(url, dest_file)
    except urllib.error.HTTPError as e:
      if e.code == 404:
        if DEBUG:
          print(f"File {file} not found (404), skipping.")
      else:
        raise

def gui_mode():
  BRANCHES_GITHUB_API_URL = "https://api.github.com/repos/MarlinFirmware/Configurations/branches"
  PATHS_GITHUB_API_URL = "https://api.github.com/repos/MarlinFirmware/Configurations/git/trees"

  url_data_cache = {}

  def add_to_cache(key, data):
    if DEBUG:
      print(f"Caching {key}")
    if key not in url_data_cache:
      url_data_cache[key] = data

  def get_from_cache(key):
    if DEBUG:
      print(f"Checking cache for {key}")
      if key in url_data_cache:
        print("Found in cache")
    return url_data_cache.get(key)

  def display_error(error):
    if DEBUG:
      print(f"Error: {error}")
    selected_label.config(text=f"Error: {error}")

  def clear_error():
    if DEBUG:
      print("Clearing error")
    selected_label.config(text="")

  def exit_gui_and_download():
    nonlocal config_path, branch, vendor, model
    if branch == "":
      display_error("Select a branch.")
      return
    if vendor == "":
      display_error("Select a vendor.")
      return
    if model == "":
      display_error("Select A Vendor.")
      return
    config_path = f"{vendor}/{model}"
    root.quit()
    root.destroy()

  def get_branches():
    if DEBUG:
      print("Fetching branches...")
    try:
      cached_data = get_from_cache(BRANCHES_GITHUB_API_URL)
      if cached_data:
        branches_data = cached_data
      else:
        response = requests.get(BRANCHES_GITHUB_API_URL)
        response.raise_for_status()
        branches_data = response.json()
        add_to_cache(BRANCHES_GITHUB_API_URL, branches_data)

        branches_data.sort(key=lambda x: x['name'], reverse=True)

        bugfix_branches = [branch for branch in branches_data if branch['name'].startswith('bugfix')]
        non_bugfix_branches = [branch for branch in branches_data if not branch['name'].startswith('bugfix')]
        branches_data = bugfix_branches + non_bugfix_branches

      ignore_list = ["import","init"]
      branches = [branch['name'] for branch in branches_data if not any(ignore in branch['name'] for ignore in ignore_list)]
    except requests.RequestException as e:
      print(f"Error fetching branches: {e}")
      exit(1)
    return branches

  def get_config_paths():
    if DEBUG:
      print("Fetching config paths...")
    try:
      selected_branch = branch_listbox.get(branch_listbox.curselection())
      cached_data = get_from_cache(f"{PATHS_GITHUB_API_URL}/{selected_branch}?recursive=1")
      if cached_data:
        paths_data = cached_data
      else:
        response = requests.get(f"{PATHS_GITHUB_API_URL}/{selected_branch}?recursive=1")
        response.raise_for_status()
        paths_data = response.json()
        add_to_cache(f"{PATHS_GITHUB_API_URL}/{selected_branch}?recursive=1", paths_data)
      config_paths = [item['path'] for item in paths_data['tree'] if 'path' in item and item['path'].startswith('config/examples') and item['path'].endswith('Configuration.h')]
      config_paths = [os.path.dirname(path).replace('config/examples/', '').replace('/Configuration.h', '') for path in config_paths]
    except requests.RequestException as e:
      print(f"Error fetching configs: {e}")
      exit(1)
    return config_paths

  def split_config_paths(config_paths):
    vendors = []
    models = []
    for path in config_paths:
      parts = path.split('/')
      if len(parts) >= 2:
        vendor = parts[0]
        model = '/'.join(parts[1:])
        if vendor not in vendors:
          vendors.append(vendor)
        models.append(model)
      else:
        vendor = path
        vendors.append(path)
        models.append('')
    return vendors, models

  def update_vendor():
    nonlocal config_paths, branch
    clear_error()
    branch = branch_listbox.get(branch_listbox.curselection())
    config_paths = get_config_paths()
    vendors, _ = split_config_paths(config_paths)
    if DEBUG:
      print('vendors:', vendors)
    vendor_listbox.delete(0, tk.END)
    for vendor in vendors:
      vendor_listbox.insert(tk.END, vendor)
    model_listbox.delete(0, tk.END)
    model_listbox.insert(tk.END, "Select A Vendor")

  def update_models():
    nonlocal model, vendor
    clear_error()
    vendor = vendor_listbox.get(vendor_listbox.curselection())
    selected_vendor = vendor_listbox.get(vendor_listbox.curselection())
    if DEBUG:
      print('selected_vendor:', selected_vendor)
    filtered_models = [current_model.replace(selected_vendor + '/', '') for current_model in config_paths if current_model.startswith(selected_vendor + '/')]
    if DEBUG:
      print('filtered_models:', filtered_models)
    model_listbox.delete(0, tk.END)
    if not filtered_models:
      model_listbox.insert(tk.END, 'NO MODELS')
    for current_model in filtered_models:
      model_listbox.insert(tk.END, current_model)
    if model_listbox.size() == 1:
      model = model_listbox.get(0)

  def store_selected_model():
    nonlocal model
    clear_error()
    model = model_listbox.get(model_listbox.curselection())

  def on_close_button():
    root.quit()
    root.destroy()
    sys.exit(0)

  branches = get_branches()
  config_paths = []
  vendors, models = ['Select A Branch'], ['Select A Vendor']
  branch = ""
  vendor = ""
  model = ""

  root = tk.Tk()
  root.title("Fetch config GUI")

  top_frame = tk.Frame(root)
  top_frame.pack(anchor='w', pady=10)
  branch_frame = tk.Frame(top_frame)
  branch_frame.pack(side=tk.LEFT, pady=10, )
  branch_label = tk.Label(branch_frame, text="Branch List:")
  branch_label.pack(side=tk.TOP, padx=5)
  branch_scrollbar = tk.Scrollbar(branch_frame, orient=tk.VERTICAL, takefocus=False)
  branch_listbox = tk.Listbox(branch_frame, width=29, yscrollcommand=branch_scrollbar.set)
  branch_listbox = tk.Listbox(branch_frame, width=29)
  branch_listbox.focus_set()
  branch_scrollbar.config(command=branch_listbox.yview)
  branch_scrollbar.pack(side=tk.RIGHT, fill=tk.Y)
  for current_branch in branches:
    branch_listbox.insert(tk.END, current_branch)
  branch_listbox.bind('<<ListboxSelect>>', lambda event: update_vendor() if branch_listbox.curselection() else None)
  branch_listbox.pack(side=tk.LEFT, padx=5)

  vendor_frame = tk.Frame(top_frame)
  vendor_frame.pack(side=tk.LEFT, pady=10)
  vendor_label = tk.Label(vendor_frame, text="Vendor List:")
  vendor_label.pack(side=tk.TOP, padx=5)
  vendor_scrollbar = tk.Scrollbar(vendor_frame, orient=tk.VERTICAL, takefocus=False)
  vendor_listbox = tk.Listbox(vendor_frame, width=29)
  vendor_listbox.config(yscrollcommand=vendor_scrollbar.set)
  vendor_scrollbar.config(command=vendor_listbox.yview)
  vendor_scrollbar.pack(side=tk.RIGHT, fill=tk.Y)
  for current_vendor in vendors:
    vendor_listbox.insert(tk.END, current_vendor)
  vendor_listbox.bind('<<ListboxSelect>>', lambda event: update_models() if vendor_listbox.curselection() else None)
  vendor_listbox.pack(side=tk.LEFT, padx=5)

  model_frame = tk.Frame(root)
  model_frame.pack(anchor='w', pady=10)
  model_label = tk.Label(model_frame, text="Printer Model List:")
  model_label.pack(side=tk.TOP, padx=5)
  model_scrollbar = tk.Scrollbar(model_frame, orient=tk.VERTICAL, takefocus=False)
  model_listbox = tk.Listbox(model_frame, width=60)
  model_listbox.config(yscrollcommand=model_scrollbar.set)
  model_scrollbar.config(command=model_listbox.yview)
  model_scrollbar.pack(side=tk.RIGHT, fill=tk.Y)
  for current_model in models:
    model_listbox.insert(tk.END, current_model)
  model_listbox.bind('<<ListboxSelect>>', lambda event: store_selected_model() if model_listbox.curselection() else None)
  model_listbox.pack(side=tk.LEFT, padx=5)

  selected_label = tk.Label(root, text="", fg="red", font=("Helvetica", 12))
  selected_label.pack(pady=5)
  fetch_button = tk.Button(root, text="Download config files", command=exit_gui_and_download)
  fetch_button.pack(pady=5)

  root.protocol("WM_DELETE_WINDOW", on_close_button)
  root.mainloop()

  config_path = vendor
  if model != "NO MODELS":
    config_path = config_path + '/' + model
  return branch, 'examples/' + config_path

def main():
  fetch_configs(*gui_mode())

if __name__ == "__main__":
  main()
