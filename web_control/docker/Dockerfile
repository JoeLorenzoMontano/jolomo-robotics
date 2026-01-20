FROM python:3.11-slim

WORKDIR /app

# Install dependencies
COPY docker/requirements.txt .
RUN pip install --no-cache-dir -r requirements.txt

# Copy application files
COPY docker/app.py .
COPY docker/templates/ templates/
COPY docker/static/ static/
COPY docker/kinematics/ kinematics/
COPY configs/ configs/

# Expose port
EXPOSE 5001

# Set environment variable to force Python to flush output
ENV PYTHONUNBUFFERED=1

# Run the application
CMD ["python", "-u", "app.py"]
