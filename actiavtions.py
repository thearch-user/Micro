import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import seaborn as sns
from sklearn.preprocessing import StandardScaler
from sklearn.decomposition import PCA
from sklearn.cluster import KMeans
from sklearn.metrics import silhouette_score
from sklearn.model_selection import train_test_split
from sklearn.ensemble import RandomForestClassifier
from sklearn.metrics import classification_report, confusion_matrix
from imblearn.over_sampling import SMOTE
from scipy.stats import zscore
import statsmodels.api as sm
from statsmodels.formula.api import ols
from statsmodels.stats.multicomp import pairwise_tukeyhsd
import warnings
warnings.filterwarnings('ignore')
import logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)    
def load_data(file_path):
    """Load dataset from a CSV file."""
    try:
        data = pd.read_csv(file_path)
        logger.info(f"Data loaded successfully from {file_path}")
        return data
    except Exception as e:
        logger.error(f"Error loading data: {e}")
        return None
def preprocess_data(data):
    """Preprocess the dataset by handling missing values and encoding categorical variables."""
    try:
        # Handle missing values
        data = data.dropna()
        # Encode categorical variables
        data = pd.get_dummies(data, drop_first=True)
        logger.info("Data preprocessing completed")
        return data
    except Exception as e:
        logger.error(f"Error in preprocessing data: {e}")
        return None
def perform_eda(data):
    """Perform exploratory data analysis."""
    # try:
        # Summary statistics
    summary = data.describe()
    logger.info("Summary statistics:\n" + str(summary))
    # Correlation matrix
    corr = data.corr()
    plt.figure(figsize=(10, 8))
    sns.heatmap(corr, annot=True, fmt=".2f", cmap='coolwarm')
    plt.title("Correlation Matrix")
    plt.show()
    logger.info("EDA completed")
except Exception as e:
        logger.error(f"Error in EDA: {e}")
def reduce_dimensions(data, n_components=2):
    """Reduce dimensions using PCA."""
    try:
        scaler = StandardScaler()
        scaled_data = scaler.fit_transform(data)
        pca = PCA(n_components=n_components)
        pca_data = pca.fit_transform(scaled_data)
        logger.info(f"PCA completed with {n_components} components")
        return pca_data
    except Exception as e:
        logger.error(f"Error in PCA: {e}")
        return None
def cluster_data(data, n_clusters=3):
    """Cluster data using KMeans."""
    try:
        kmeans = KMeans(n_clusters=n_clusters, random_state=42)
        clusters = kmeans.fit_predict(data)
        silhouette_avg = silhouette_score(data, clusters)
        logger.info(f"KMeans clustering completed with {n_clusters} clusters, Silhouette Score: {silhouette_avg}")
        return clusters
    except Exception as e:
        logger.error(f"Error in clustering: {e}")
        return None
def classify_data(data, target_column):
    """Classify data using Random Forest."""
    try:
        X = data.drop(columns=[target_column])
        y = data[target_column]
        X_train, X_test, y_train, y_test = train_test_split(X, y, test_size=0.3, random_state=42)
        smote = SMOTE(random_state=42)
        X_train_res, y_train_res = smote.fit_resample(X_train, y_train)
        clf = RandomForestClassifier(random_state=42)
        clf.fit(X_train_res, y_train_res)
        y_pred = clf.predict(X_test)
        report = classification_report(y_test, y_pred)
        cm = confusion_matrix(y_test, y_pred)
        logger.info("Classification Report:\n" + report)
        logger.info("Confusion Matrix:\n" + str(cm))
    except Exception as e:
        logger.error(f"Error in classification: {e}")
def perform_anova(data, dependent_var, independent_var):
    """Perform ANOVA and Tukey's HSD test."""
    try:
        model = ols(f'{dependent_var} ~ C({independent_var})', data=data).fit()
        anova_table = sm.stats.anova_lm(model, typ=2)
        logger.info("ANOVA Table:\n" + str(anova_table))
        tukey = pairwise_tukeyhsd(endog=data[dependent_var], groups=data[independent_var], alpha=0.05)
        logger.info("Tukey's HSD results:\n" + str(tukey))
    except Exception as e:
        logger.error(f"Error in ANOVA: {e}")
        