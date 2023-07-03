from rich.console import Console
from rich.menu import Menu
from rich.prompt import Prompt
from rich.table import Table

# Initialize the console
console = Console()

# Define the menu options
menu_options = [
    {"name": "Option 1", "value": 1},
    {"name": "Option 2", "value": 2},
    {"name": "Option 3", "value": 3},
    {"name": "Exit", "value": 0},
]

# Define the menu table
table = Table(title="Menu")
table.add_column("Option", justify="center")
table.add_column("Name", justify="center")
for option in menu_options:
    table.add_row(f"[yellow]{option['value']}[/yellow]", option["name"])

# Display the menu and prompt the user for input
while True:
    console.print(table)
    choice = Prompt.ask("Enter your choice:", choices=[str(option["value"]) for option in menu_options] + ["0"])

    # Handle the user's choice
    if choice == "0":
        console.print("Exiting...")
        break
    elif choice.isdigit() and int(choice) in [option["value"] for option in menu_options]:
        console.print(f"You chose [yellow]{menu_options[int(choice)-1]['name']}[/yellow].")
    else:
        console.print("[red]Invalid choice. Please try again.[/red]")